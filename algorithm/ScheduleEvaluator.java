package algorithm;

import model.DroneTrip;
import model.EVRoute;
import model.Node;
import model.Solution;
import util.DataLoader;
import util.EnergyUtil;
import util.TimeUtil;

import java.util.*;

/**
 * Single source of truth for the paper schedule and objective.
 *
 * The former decoder maintained several partially stale copies of EV and drone
 * time.  This evaluator derives all route times, drone flight arrivals, hover,
 * lifecycle endpoints, carried arcs, energy, and the four terms of Eq.(1) from
 * the current solution.  It deliberately contains no search policy.
 */
public final class ScheduleEvaluator {
    private static final double EPS = 1e-9;
    private static final int MAX_FIXED_POINT_ITERATIONS = 48;

    private ScheduleEvaluator() {
    }

    public static void evaluate(Solution solution) {
        if (solution == null) return;

        for (int iteration = 0; iteration < MAX_FIXED_POINT_ITERATIONS; iteration++) {
            recomputeRoutes(solution);
            boolean changed = recomputeDroneTrips(solution);
            if (!changed) break;
        }
        // One final route pass incorporates the final rendezvous values.
        recomputeRoutes(solution);
        recomputeDroneTrips(solution);
        rebuildLifecycleAndCarriedArcs(solution);
        computeObjective(solution);
    }

    public static void computeObjective(Solution solution) {
        double cost = 0.0;
        Set<Integer> usedDrones = new HashSet<>();

        for (EVRoute route : solution.evRoutes) {
            cost += Constants.EV_DISPATCH_COST;
            cost += route.energyUsed * Constants.ELECTRICITY_PRICE;
            route.totalCost = Constants.EV_DISPATCH_COST
                    + route.energyUsed * Constants.ELECTRICITY_PRICE;
        }
        for (DroneTrip trip : solution.allDroneTrips) {
            usedDrones.add(trip.droneId);
            cost += trip.energyUsed * Constants.ELECTRICITY_PRICE;
        }
        cost += usedDrones.size() * Constants.DRONE_DISPATCH_COST;
        solution.totalCost = cost;
    }

    private static void recomputeRoutes(Solution solution) {
        for (EVRoute route : solution.evRoutes) {
            Node start = DataLoader.getDepot(route.startDepotId);
            Node position = start;
            double time = Constants.T_START;
            double remainingLoad = route.totalDemandServedByEVAndDrone();
            double energy = 0.0;

            route.arrivalTimes.clear();
            route.departureTimes.clear();
            route.loads.clear();
            route.arrivalTimeByCustomerNode.clear();
            route.departureTimeByCustomerNode.clear();

            route.arrivalTimes.add(time);
            /*
             * Kept deliberately, unlike the customer-node case above: the "EV services
             * take precedence" rule is about the service time at a *customer* node, and a
             * depot has no such window to fit a landing into -- Eq.6 only requires the
             * departure to be at or after Ts.  This models the paper's Fig.1c, where an EV
             * picks up a drone that has already flown a depot trip.
             */
            time = Math.max(time, requiredRendezvousAt(solution, route.evId, route.startDepotId));
            time += route.launchCountAtNode(route.startDepotId)
                    * (Constants.DRONE_T1 + Constants.DRONE_T2);
            route.departureTimes.add(time);

            for (int customerId : route.customerIds) {
                Node customer = DataLoader.getCustomer(customerId);
                double distance = DataLoader.distance(position.id, customer.id) * Constants.DETOUR_COEFF;
                double travel = TimeUtil.travelTime(time, distance);
                double arrival = time + travel;
                energy += EnergyUtil.evEnergy(time, travel, Math.max(0.0, remainingLoad));

                /*
                 * Section 3: "unlike vehicles that must wait for the return of the drone
                 * at the rendezvous node, this paper stipulates that EV services take
                 * precedence over drone operation".  The EV therefore leaves as soon as
                 * its own service and drone launch/retrieve operations are done -- it
                 * never stretches its stay to wait for a late drone.  A drone that cannot
                 * land within that window makes the trip infeasible, which is exactly
                 * what ConstraintChecker reports as Eq.26.
                 */
                double departure = arrival + Constants.EV_SERVICE_TIME
                        + route.launchCountAtNode(customerId)
                        * (Constants.DRONE_T1 + Constants.DRONE_T2);

                remainingLoad -= customer.demand;
                remainingLoad -= droneDemandLaunchedAt(route, customerId);

                route.arrivalTimes.add(arrival);
                route.departureTimes.add(departure);
                route.arrivalTimeByCustomerNode.put(customerId, arrival);
                route.departureTimeByCustomerNode.put(customerId, departure);
                route.loads.add(Math.max(0.0, remainingLoad));

                time = departure;
                position = customer;
            }

            double backDistance = DataLoader.distance(position.id, route.endDepotId)
                    * Constants.DETOUR_COEFF;
            double backTravel = TimeUtil.travelTime(time, backDistance);
            energy += EnergyUtil.evEnergy(time, backTravel, 0.0);
            route.arrivalTimes.add(time + backTravel);
            route.energyUsed = energy;
            route.feasible = true;
        }
    }

    private static double requiredRendezvousAt(Solution solution, int evId, int nodeId) {
        double required = Constants.T_START;
        for (DroneTrip trip : solution.allDroneTrips) {
            if (trip.retrieveEVId == evId && trip.retrieveNodeId == nodeId) {
                required = Math.max(required, trip.rendezvousTime);
            }
        }
        return required;
    }

    private static boolean recomputeDroneTrips(Solution solution) {
        boolean changed = false;
        Map<Integer, EVRoute> routeById = routesById(solution);
        Map<Integer, List<DroneTrip>> byDrone = tripsByDrone(solution);

        for (List<DroneTrip> trips : byDrone.values()) {
            DroneTrip previous = null;
            for (DroneTrip trip : trips) {
                double oldDeparture = trip.departTime;
                double oldFlightArrival = trip.flightArrivalTime;
                double oldRendezvous = trip.rendezvousTime;
                double oldHover = trip.hoverTime;

                double departure;
                if (trip.launchEVId > 0) {
                    EVRoute launchRoute = routeById.get(trip.launchEVId);
                    departure = launchRoute == null ? Double.POSITIVE_INFINITY
                            : launchRoute.getDepartureAtNode(trip.launchNodeId);
                } else {
                    departure = Constants.T_START;
                }
                if (previous != null) {
                    departure = Math.max(departure, previous.availableAfterServiceTime);
                }

                double d1 = DataLoader.distance(trip.launchNodeId, trip.serveNodeId);
                double d2 = DataLoader.distance(trip.serveNodeId, trip.retrieveNodeId);
                double flightArrival = departure + TimeUtil.droneTravelTime(d1 + d2)
                        + 4.0 * Constants.DRONE_T1;
                double rendezvous = flightArrival;
                double hover = 0.0;

                if (trip.retrieveEVId > 0) {
                    EVRoute retrieveRoute = routeById.get(trip.retrieveEVId);
                    if (retrieveRoute != null) {
                        double evArrival = retrieveRoute.getArrivalAtNode(trip.retrieveNodeId);
                        hover = Math.max(0.0, evArrival - flightArrival);
                        // A drone that arrives during EV service can land before
                        // departure without extra hover; an earlier one hovers.
                        rendezvous = Math.max(flightArrival, evArrival);
                    }
                }

                trip.departTime = departure;
                trip.flightArrivalTime = flightArrival;
                trip.rendezvousTime = rendezvous;
                trip.arriveTime = rendezvous;
                trip.hoverTime = hover;
                trip.availableAfterServiceTime = trip.retrieveEVId < 0
                        ? rendezvous + Constants.DRONE_T1 + Constants.DRONE_T2
                        : rendezvous;
                trip.energyUsed = EnergyUtil.droneEnergy(d1, d2, hover);

                if (different(oldDeparture, departure) || different(oldFlightArrival, flightArrival)
                        || different(oldRendezvous, rendezvous) || different(oldHover, hover)) {
                    changed = true;
                }
                previous = trip;
            }
        }
        return changed;
    }

    private static boolean different(double a, double b) {
        return Double.isInfinite(a) != Double.isInfinite(b) || Math.abs(a - b) > EPS;
    }

    private static void rebuildLifecycleAndCarriedArcs(Solution solution) {
        for (EVRoute route : solution.evRoutes) route.carriedDroneByArc.clear();
        Map<Integer, EVRoute> routes = routesById(solution);

        for (List<DroneTrip> trips : tripsByDrone(solution).values()) {
            if (trips.isEmpty()) continue;
            DroneTrip first = trips.get(0);
            int dispatchDepot = first.launchEVId > 0
                    ? depotForLaunch(routes.get(first.launchEVId), first.launchNodeId)
                    : first.launchNodeId;

            for (DroneTrip trip : trips) trip.dispatchDepotId = dispatchDepot;

            if (first.launchEVId > 0) {
                EVRoute route = routes.get(first.launchEVId);
                if (route != null) markCarried(route, route.startDepotId, first.launchNodeId, first.droneId);
            }

            for (int i = 0; i < trips.size(); i++) {
                DroneTrip trip = trips.get(i);
                DroneTrip next = i + 1 < trips.size() ? trips.get(i + 1) : null;
                if (trip.retrieveEVId > 0) {
                    EVRoute ownerRoute = routes.get(trip.retrieveEVId);
                    if (ownerRoute != null) {
                        if (next != null && next.launchEVId == trip.retrieveEVId) {
                            markCarried(ownerRoute, trip.retrieveNodeId, next.launchNodeId, trip.droneId);
                        } else if (next == null) {
                            markCarried(ownerRoute, trip.retrieveNodeId, ownerRoute.endDepotId, trip.droneId);
                            trip.returnDepotId = ownerRoute.endDepotId;
                        }
                    }
                } else if (next == null) {
                    trip.returnDepotId = trip.retrieveNodeId;
                }
            }

            DroneTrip last = trips.get(trips.size() - 1);
            int returnDepot = last.retrieveEVId > 0
                    ? last.returnDepotId : last.retrieveNodeId;
            for (DroneTrip trip : trips) trip.returnDepotId = returnDepot;
        }
    }

    private static int depotForLaunch(EVRoute route, int launchNodeId) {
        return route == null ? -1 : route.startDepotId;
    }

    private static void markCarried(EVRoute route, int fromNodeId, int toNodeId, int droneId) {
        List<Integer> nodes = new ArrayList<>();
        nodes.add(route.startDepotId);
        nodes.addAll(route.customerIds);
        nodes.add(route.endDepotId);

        int from = fromNodeId == route.startDepotId ? 0 : nodes.indexOf(fromNodeId);
        int to = toNodeId == route.endDepotId ? nodes.size() - 1 : nodes.indexOf(toNodeId);
        if (from < 0 || to < 0 || to < from) return;
        for (int i = from; i < to; i++) {
            route.carriedDroneByArc.put(EVRoute.arcKey(nodes.get(i), nodes.get(i + 1)), droneId);
        }
    }

    private static double droneDemandLaunchedAt(EVRoute route, int nodeId) {
        double demand = 0.0;
        for (DroneTrip trip : route.droneTrips) {
            if (trip.launchEVId == route.evId && trip.launchNodeId == nodeId) {
                demand += DataLoader.getCustomer(trip.serveNodeId).demand;
            }
        }
        return demand;
    }

    public static Map<Integer, EVRoute> routesById(Solution solution) {
        Map<Integer, EVRoute> routes = new HashMap<>();
        for (EVRoute route : solution.evRoutes) routes.put(route.evId, route);
        return routes;
    }

    public static Map<Integer, List<DroneTrip>> tripsByDrone(Solution solution) {
        Map<Integer, List<DroneTrip>> result = new HashMap<>();
        for (DroneTrip trip : solution.allDroneTrips) {
            result.computeIfAbsent(trip.droneId, k -> new ArrayList<>()).add(trip);
        }
        for (List<DroneTrip> trips : result.values()) {
            trips.sort(Comparator.comparingInt(t -> t.tripIndex));
        }
        return result;
    }
}
