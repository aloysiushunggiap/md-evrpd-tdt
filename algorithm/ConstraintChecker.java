package algorithm;

import model.DroneTrip;
import model.EVRoute;
import model.Node;
import model.Solution;
import util.DataLoader;
import util.TimeUtil;

import java.util.*;

/**
 * Equation-oriented feasibility checker for the paper-compliant baseline.
 * Every public evaluation first recalculates the same schedule used by the
 * objective, eliminating stale EV/drone timestamps.
 */
public final class ConstraintChecker {
    private static final double EPS = 1e-8;

    private ConstraintChecker() {
    }

    public static String checkAll(Solution solution) {
        List<String> violations = collectViolations(solution);
        return violations.isEmpty() ? null : violations.get(0);
    }

    public static boolean isFeasible(Solution solution) {
        return collectViolations(solution).isEmpty();
    }

    /** Diagnostic only. ICGA uses feasibility, not a penalty surrogate. */
    public static double penalty(Solution solution) {
        if (solution == null) return 1e12;
        return collectViolations(solution).size() * Constants.PENALTY_CONTINUITY;
    }

    public static List<String> collectViolations(Solution solution) {
        List<String> out = new ArrayList<>();
        if (solution == null) {
            out.add("Solution is null");
            return out;
        }
        try {
            ScheduleEvaluator.evaluate(solution);
        } catch (RuntimeException e) {
            out.add("Schedule evaluation failed: " + e.getMessage());
            return out;
        }

        Map<Integer, EVRoute> routeById = ScheduleEvaluator.routesById(solution);
        Map<Integer, List<DroneTrip>> tripsByDrone = ScheduleEvaluator.tripsByDrone(solution);
        Map<Integer, Integer> served = servedCustomerCount(solution);
        Map<Integer, Integer> depotEvOut = depotCounts();
        Map<Integer, Integer> depotEvIn = depotCounts();
        Map<Integer, Integer> depotDroneOut = depotCounts();
        Map<Integer, Integer> depotDroneIn = depotCounts();
        Map<Integer, Integer> launchesAtCustomer = new HashMap<>();
        Map<Integer, Integer> retrievesAtCustomer = new HashMap<>();
        Set<Integer> droneServed = new HashSet<>();

        Set<Integer> evIds = new HashSet<>();
        for (EVRoute route : solution.evRoutes) {
            if (!evIds.add(route.evId)) out.add("Eq.3: duplicate EV id " + route.evId);
            depotEvOut.merge(route.startDepotId, 1, Integer::sum);
            depotEvIn.merge(route.endDepotId, 1, Integer::sum);
            validateEvRoute(solution, route, out);
        }
        for (Node depot : DataLoader.depots) {
            if (!Objects.equals(depotEvOut.get(depot.id), depotEvIn.get(depot.id))) {
                out.add("Eq.2: EV dispatch/return imbalance at depot " + depot.id);
            }
        }

        for (Node customer : DataLoader.customers) {
            int count = served.getOrDefault(customer.id, 0);
            if (count != 1) out.add("Eq.8: customer " + customer.id + " served " + count + " times");
        }

        for (List<DroneTrip> trips : tripsByDrone.values()) {
            if (trips.isEmpty()) continue;
            DroneTrip first = trips.get(0);
            if (!DataLoader.depotMap.containsKey(first.dispatchDepotId)) {
                out.add("Eq.40: drone " + first.droneId + " has no depot dispatch origin");
            } else {
                depotDroneOut.merge(first.dispatchDepotId, 1, Integer::sum);
            }
            for (int i = 0; i < trips.size(); i++) {
                DroneTrip trip = trips.get(i);
                validateDroneTrip(routeById, trip, droneServed, launchesAtCustomer,
                        retrievesAtCustomer, out);
                if (i > 0) validateContinuity(trips.get(i - 1), trip, routeById, out);
            }
            DroneTrip last = trips.get(trips.size() - 1);
            if (!DataLoader.depotMap.containsKey(last.returnDepotId)) {
                out.add("Eq.4/5: drone " + last.droneId + " does not return to a depot");
            } else {
                depotDroneIn.merge(last.returnDepotId, 1, Integer::sum);
                if (finalReturnTime(last, routeById) > Constants.T_END + EPS) {
                    out.add("Eq.25: drone " + last.droneId + " returns after depot closing");
                }
            }
        }
        for (Node depot : DataLoader.depots) {
            if (!Objects.equals(depotDroneOut.get(depot.id), depotDroneIn.get(depot.id))) {
                out.add("Eq.4: drone dispatch/return imbalance at depot " + depot.id);
            }
        }

        for (Map.Entry<Integer, Integer> entry : launchesAtCustomer.entrySet()) {
            int nodeId = entry.getKey();
            int launches = entry.getValue();
            int retrieves = retrievesAtCustomer.getOrDefault(nodeId, 0);
            if (droneServed.contains(nodeId)) out.add("Eq.22: drone-served customer " + nodeId + " launches a drone");
            if (launches > 2) out.add("Eq.29: more than two launches at customer " + nodeId);
            if (launches == 2 && retrieves != 1 && !hasInboundDrone(routeById, nodeId)) {
                out.add("Eq.28/29: two launches at " + nodeId + " lack a carried/retrieved drone");
            }
        }
        for (Map.Entry<Integer, Integer> entry : retrievesAtCustomer.entrySet()) {
            int nodeId = entry.getKey();
            if (droneServed.contains(nodeId)) out.add("Eq.23: drone-served customer " + nodeId + " retrieves a drone");
            if (entry.getValue() > 1) out.add("Eq.27: more than one retrieval at customer " + nodeId);
        }
        validateCarriedArcs(solution, out);
        return out;
    }

    private static void validateEvRoute(Solution solution, EVRoute route, List<String> out) {
        if (!DataLoader.depotMap.containsKey(route.startDepotId)
                || !DataLoader.depotMap.containsKey(route.endDepotId)) {
            out.add("Eq.2/3: EV" + route.evId + " uses an invalid depot");
            return;
        }
        if (route.totalDemandServedByEVAndDrone() > Constants.EV_CAPACITY + EPS) {
            out.add("Eq.9: EV" + route.evId + " exceeds payload capacity");
        }
        if (route.energyUsed > Constants.EV_BATTERY - Constants.EV_MIN_ENERGY + EPS) {
            out.add("Eq.10: EV" + route.evId + " drops below minimum SoC");
        }
        if (route.arrivalTimes.size() != route.customerIds.size() + 2
                || route.departureTimes.size() != route.customerIds.size() + 1) {
            out.add("Eq.11/12: EV" + route.evId + " has inconsistent timing vectors");
            return;
        }
        if (route.departureTimes.get(0) + EPS < Constants.T_START) out.add("Eq.6: EV leaves before Ts");
        if (route.arrivalTimes.get(route.arrivalTimes.size() - 1) > Constants.T_END + EPS) out.add("Eq.7: EV returns after Tf");

        List<Integer> nodes = new ArrayList<>();
        nodes.add(route.startDepotId);
        nodes.addAll(route.customerIds);
        nodes.add(route.endDepotId);
        Set<Integer> localCustomers = new HashSet<>();
        for (int i = 0; i < route.customerIds.size(); i++) {
            int customerId = route.customerIds.get(i);
            if (!localCustomers.add(customerId)) out.add("Eq.14: EV visits customer twice");
            double expectedArrival = route.departureTimes.get(i) + TimeUtil.travelTime(
                    route.departureTimes.get(i),
                    DataLoader.distance(nodes.get(i), nodes.get(i + 1)) * Constants.DETOUR_COEFF);
            if (Math.abs(expectedArrival - route.arrivalTimes.get(i + 1)) > 1e-6) {
                out.add("Eq.11/12: EV travel-time equality violated");
            }
            double minimumDeparture = route.arrivalTimes.get(i + 1) + Constants.EV_SERVICE_TIME
                    + route.launchCountAtNode(customerId) * (Constants.DRONE_T1 + Constants.DRONE_T2);
            if (route.departureTimes.get(i + 1) + EPS < minimumDeparture) {
                out.add("Eq.13: EV leaves customer too early");
            }
        }
        int last = route.customerIds.size();
        double expectedEnd = route.departureTimes.get(last) + TimeUtil.travelTime(
                route.departureTimes.get(last),
                DataLoader.distance(nodes.get(last), nodes.get(last + 1)) * Constants.DETOUR_COEFF);
        if (Math.abs(expectedEnd - route.arrivalTimes.get(last + 1)) > 1e-6) {
            out.add("Eq.11/12: EV final travel-time equality violated");
        }
    }

    private static void validateDroneTrip(Map<Integer, EVRoute> routes, DroneTrip trip,
                                          Set<Integer> droneServed, Map<Integer, Integer> launches,
                                          Map<Integer, Integer> retrieves, List<String> out) {
        try {
            Node served = DataLoader.getCustomer(trip.serveNodeId);
            DataLoader.getNode(trip.launchNodeId);
            DataLoader.getNode(trip.retrieveNodeId);
            droneServed.add(trip.serveNodeId);
            if (served.demand > Constants.DRONE_CAPACITY + EPS) out.add("Eq.35: drone payload exceeds capacity");
            if (trip.departTime + EPS < Constants.T_START) out.add("Eq.24: drone departs before Ts");
            double d1 = DataLoader.distance(trip.launchNodeId, trip.serveNodeId);
            double d2 = DataLoader.distance(trip.serveNodeId, trip.retrieveNodeId);
            double expectedFlightArrival = trip.departTime + TimeUtil.droneTravelTime(d1 + d2)
                    + 4.0 * Constants.DRONE_T1;
            if (Math.abs(expectedFlightArrival - trip.flightArrivalTime) > 1e-6) {
                out.add("Eq.30/31: drone flight-time equality violated");
            }
            if (trip.energyUsed > Constants.DRONE_BATTERY - Constants.DRONE_MIN_ENERGY + EPS) {
                out.add("Eq.18: drone drops below minimum SoC");
            }
            if (trip.launchEVId > 0) {
                EVRoute route = routes.get(trip.launchEVId);
                if (route == null || !route.visitsNode(trip.launchNodeId)) out.add("Eq.19/37: launch EV misses node");
                else {
                    launches.merge(trip.launchNodeId, 1, Integer::sum);
                    if (trip.departTime + EPS < route.getArrivalAtNode(trip.launchNodeId)
                            + Constants.DRONE_T1 + Constants.DRONE_T2) {
                        out.add("Eq.34: launch before EV operation completes");
                    }
                }
            } else if (!DataLoader.depotMap.containsKey(trip.launchNodeId)) {
                out.add("Eq.40: initial drone launch is not from depot/EV");
            }
            if (trip.retrieveEVId > 0) {
                EVRoute route = routes.get(trip.retrieveEVId);
                if (route == null || !route.visitsNode(trip.retrieveNodeId)) out.add("Eq.20/21: retrieve EV misses node");
                else {
                    retrieves.merge(trip.retrieveNodeId, 1, Integer::sum);
                    if (trip.rendezvousTime > route.getDepartureAtNode(trip.retrieveNodeId) + EPS) out.add("Eq.26: late retrieval");
                    double expectedHover = Math.max(0.0, route.getArrivalAtNode(trip.retrieveNodeId) - trip.flightArrivalTime);
                    if (Math.abs(expectedHover - trip.hoverTime) > 1e-6) out.add("Eq.1: hover inconsistent with schedule");
                }
            } else if (!DataLoader.depotMap.containsKey(trip.retrieveNodeId)) {
                out.add("Eq.5: non-EV retrieval is not a depot");
            }
        } catch (IllegalArgumentException e) {
            out.add("Invalid drone trip: " + e.getMessage());
        }
    }

    private static void validateContinuity(DroneTrip previous, DroneTrip next,
                                           Map<Integer, EVRoute> routes, List<String> out) {
        if (next.tripIndex != previous.tripIndex + 1) out.add("Eq.41: non-contiguous trip index");
        if (next.departTime + EPS < previous.availableAfterServiceTime) out.add("Eq.32/41: drone relaunches too early");
        if (previous.retrieveEVId > 0) {
            if (next.launchEVId != previous.retrieveEVId || next.launchNodeId != previous.retrieveNodeId) {
                out.add("Eq.16/38/41: EV-carried drone continuity is broken");
            }
        } else if (next.launchEVId >= 0 || next.launchNodeId != previous.retrieveNodeId) {
            out.add("Eq.32/41: depot drone continuity is broken");
        }
    }

    private static void validateCarriedArcs(Solution solution, List<String> out) {
        for (EVRoute route : solution.evRoutes) {
            for (Map.Entry<String, Integer> entry : route.carriedDroneByArc.entrySet()) {
                if (entry.getValue() == null || entry.getValue() <= 0) out.add("Eq.15: invalid carried drone arc");
            }
            for (DroneTrip trip : solution.allDroneTrips) {
                if (trip.launchEVId != route.evId || DataLoader.depotMap.containsKey(trip.launchNodeId)) continue;
                int index = route.customerIds.indexOf(trip.launchNodeId);
                boolean inbound = index >= 0 && route.carriedDroneOnArc(
                        index == 0 ? route.startDepotId : route.customerIds.get(index - 1),
                        trip.launchNodeId) != null;
                boolean sameNodeRetrieve = solution.allDroneTrips.stream().anyMatch(other ->
                        other.droneId == trip.droneId && other.tripIndex == trip.tripIndex - 1
                                && other.retrieveEVId == route.evId && other.retrieveNodeId == trip.launchNodeId);
                if (!inbound && !sameNodeRetrieve) out.add("Eq.16/17/38: drone not carried to launch node");
            }
        }
    }

    private static boolean hasInboundDrone(Map<Integer, EVRoute> routes, int nodeId) {
        for (EVRoute route : routes.values()) {
            int index = route.customerIds.indexOf(nodeId);
            if (index >= 0) {
                int previous = index == 0 ? route.startDepotId : route.customerIds.get(index - 1);
                if (route.carriedDroneOnArc(previous, nodeId) != null) return true;
            }
        }
        return false;
    }

    private static Map<Integer, Integer> servedCustomerCount(Solution solution) {
        Map<Integer, Integer> result = new HashMap<>();
        for (EVRoute route : solution.evRoutes) for (int id : route.customerIds) result.merge(id, 1, Integer::sum);
        for (DroneTrip trip : solution.allDroneTrips) result.merge(trip.serveNodeId, 1, Integer::sum);
        return result;
    }

    private static Map<Integer, Integer> depotCounts() {
        Map<Integer, Integer> counts = new HashMap<>();
        for (Node depot : DataLoader.depots) counts.put(depot.id, 0);
        return counts;
    }

    private static double finalReturnTime(DroneTrip last, Map<Integer, EVRoute> routes) {
        if (last.retrieveEVId < 0) return last.rendezvousTime;
        EVRoute route = routes.get(last.retrieveEVId);
        return route == null ? Double.POSITIVE_INFINITY : route.arrivalTimes.get(route.arrivalTimes.size() - 1);
    }

    public static EVRoute findRoute(Solution solution, int evId) {
        return ScheduleEvaluator.routesById(solution).get(evId);
    }
}
