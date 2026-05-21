package algorithm;

import model.DroneTrip;
import model.EVRoute;
import model.Node;
import model.Solution;
import util.DataLoader;

import java.util.*;

/**
 * Kiểm tra ràng buộc cho MD-EVRPD-TDT.
 *
 * Bám các nhóm constraint chính trong paper:
 * - customer served exactly once
 * - depot EV balance
 * - EV capacity / SoC / time window
 * - drone payload / SoC / time window
 * - launch/retrieve node validity
 * - launch/retrieve count at customer node
 * - synchronization at retrieve node
 * - drone trip continuity
 */
public class ConstraintChecker {

    private ConstraintChecker() {
    }

    public static String checkAll(Solution sol) {
        if (sol == null) return "Solution is null";
        List<String> violations = collectViolations(sol);
        return violations.isEmpty() ? null : violations.get(0);
    }

    public static boolean isFeasible(Solution sol) {
        return collectViolations(sol).isEmpty();
    }

    /**
     * Penalty mềm để ICGA có gradient.
     * Không thay thế ràng buộc paper, chỉ giúp heuristic phân biệt nghiệm xấu/tốt.
     */
    public static double penalty(Solution sol) {
        if (sol == null) return 1e12;

        double p = 0.0;

        // ======================================================
        // Eq.(8): each customer served exactly once
        // ======================================================
        Map<Integer, Integer> served = servedCustomerCount(sol);

        for (Node c : DataLoader.customers) {
            int count = served.getOrDefault(c.id, 0);
            if (count == 0) {
                p += Constants.PENALTY_MISSED_CUSTOMER;
            } else if (count > 1) {
                p += (count - 1) * Constants.PENALTY_DUPLICATE_CUSTOMER;
            }
        }

        // ======================================================
        // Eq.(2): depot EV balance
        // ======================================================
        Map<Integer, Integer> depotOut = new HashMap<>();
        Map<Integer, Integer> depotIn = new HashMap<>();

        for (Node d : DataLoader.depots) {
            depotOut.put(d.id, 0);
            depotIn.put(d.id, 0);
        }

        for (EVRoute r : sol.evRoutes) {
            depotOut.merge(r.startDepotId, 1, Integer::sum);
            depotIn.merge(r.endDepotId, 1, Integer::sum);
        }

        for (Node d : DataLoader.depots) {
            int out = depotOut.getOrDefault(d.id, 0);
            int in = depotIn.getOrDefault(d.id, 0);
            p += Math.abs(out - in) * Constants.PENALTY_DEPOT_BALANCE;
        }

        // ======================================================
        // EV route constraints
        // ======================================================
        for (EVRoute route : sol.evRoutes) {
            // Eq.(9): EV payload includes EV customers + drone customers launched from this EV
            double demandExcess = Math.max(0.0,
                    route.totalDemandServedByEVAndDrone() - Constants.EV_CAPACITY);
            p += demandExcess * Constants.PENALTY_EV_CAPACITY;

            // Eq.(10): EV battery SoC
            double maxEnergyUse = Constants.EV_BATTERY - Constants.EV_MIN_ENERGY;
            double energyExcess = Math.max(0.0, route.energyUsed - maxEnergyUse);
            p += energyExcess * Constants.PENALTY_EV_ENERGY;

            // Eq.(6)
            if (!route.departureTimes.isEmpty()) {
                double departDepot = route.departureTimes.get(0);
                if (departDepot < Constants.T_START - 1e-9) {
                    p += (Constants.T_START - departDepot) * Constants.PENALTY_TIME;
                }
            }

            // Eq.(7)
            if (!route.arrivalTimes.isEmpty()) {
                double returnTime = route.arrivalTimes.get(route.arrivalTimes.size() - 1);
                if (returnTime > Constants.T_END + 1e-9) {
                    p += (returnTime - Constants.T_END) * Constants.PENALTY_TIME;
                }
            }

            // Eq.(27), Eq.(29): node launch/retrieve limits
            Map<Integer, Integer> launchCount = launchCountByNode(route);
            Map<Integer, Integer> retrieveCount = retrieveCountByNode(route);

            for (Map.Entry<Integer, Integer> e : launchCount.entrySet()) {
                Node node = DataLoader.getNode(e.getKey());

                // At customer node: at most two launches if EV carries/retrieves drone.
                // Conservative heuristic: absolute upper bound 2.
                if (!node.isDepot && e.getValue() > 2) {
                    p += (e.getValue() - 2) * Constants.PENALTY_NODE_OPERATION;
                }
            }

            for (Map.Entry<Integer, Integer> e : retrieveCount.entrySet()) {
                Node node = DataLoader.getNode(e.getKey());

                // Eq.(27): up to one drone can be retrieved at a customer node
                if (!node.isDepot && e.getValue() > 1) {
                    p += (e.getValue() - 1) * Constants.PENALTY_NODE_OPERATION;
                }
            }

            if (!routeDroneCarryFeasible(route)) {
                p += Constants.PENALTY_NODE_OPERATION * 2.0;
            }
        }

        // ======================================================
        // Drone constraints
        // ======================================================
        for (DroneTrip dt : sol.allDroneTrips) {
            Node servedNode = DataLoader.getCustomer(dt.serveNodeId);

            // Eq.(35): payload
            if (servedNode.demand > Constants.DRONE_CAPACITY + 1e-9) {
                p += (servedNode.demand - Constants.DRONE_CAPACITY)
                        * Constants.PENALTY_DRONE_ENERGY * 10.0;
            }

            // Eq.(18): drone battery SoC
            double remain = Constants.DRONE_BATTERY - dt.energyUsed;
            if (remain < Constants.DRONE_MIN_ENERGY - 1e-9) {
                p += (Constants.DRONE_MIN_ENERGY - remain)
                        * Constants.PENALTY_DRONE_ENERGY * 100.0;
            }

            // Eq.(24), Eq.(25)
            if (dt.departTime < Constants.T_START - 1e-9) {
                p += (Constants.T_START - dt.departTime) * Constants.PENALTY_TIME;
            }

            if (dt.arriveTime > Constants.T_END + 1e-9) {
                p += (dt.arriveTime - Constants.T_END) * Constants.PENALTY_TIME;
            }

            // Launch EV validity
            if (dt.launchEVId > 0) {
                EVRoute launchRoute = findRoute(sol, dt.launchEVId);
                if (launchRoute == null || !launchRoute.visitsNode(dt.launchNodeId)) {
                    p += Constants.PENALTY_SYNC;
                }
            }

            // Retrieve EV validity + Eq.(26)
            if (dt.retrieveEVId > 0) {
                EVRoute retrieveRoute = findRoute(sol, dt.retrieveEVId);
                if (retrieveRoute == null || !retrieveRoute.visitsNode(dt.retrieveNodeId)) {
                    p += Constants.PENALTY_SYNC;
                } else {
                    double evDepartureAtRetrieve = retrieveRoute.getDepartureAtNode(dt.retrieveNodeId);
                    if (dt.arriveTime > evDepartureAtRetrieve + 1e-9) {
                        p += (dt.arriveTime - evDepartureAtRetrieve) * Constants.PENALTY_SYNC;
                    }
                }
            }
        }

        // ======================================================
        // Eq.(41): drone trip continuity
        // ======================================================
        Map<Integer, List<DroneTrip>> byDrone = tripsByDrone(sol);
        for (List<DroneTrip> trips : byDrone.values()) {
            trips.sort(Comparator.comparingInt(t -> t.tripIndex));

            for (int i = 1; i < trips.size(); i++) {
                DroneTrip prev = trips.get(i - 1);
                DroneTrip next = trips.get(i);

                // time continuity
                if (next.departTime + 1e-9 < prev.arriveTime) {
                    p += Constants.PENALTY_CONTINUITY;
                }

                // location continuity:
                // next launch node should be the previous retrieve node for same drone
                if (next.launchNodeId != prev.retrieveNodeId) {
                    p += Constants.PENALTY_CONTINUITY;
                }

                // owner continuity:
                // if previous retrieved by EV k, next launch should be from EV k or depot only if retrieve node is depot
                if (prev.retrieveEVId > 0) {
                    if (next.launchEVId != prev.retrieveEVId) {
                        p += Constants.PENALTY_CONTINUITY;
                    }
                } else {
                    if (next.launchEVId > 0 && !DataLoader.getNode(prev.retrieveNodeId).isDepot) {
                        p += Constants.PENALTY_CONTINUITY;
                    }
                }
            }
        }

        return p;
    }

    public static List<String> collectViolations(Solution sol) {
        List<String> out = new ArrayList<>();

        if (sol == null) {
            out.add("Solution is null");
            return out;
        }

        // ======================================================
        // Eq.(8): customer served exactly once
        // ======================================================
        Map<Integer, Integer> served = servedCustomerCount(sol);

        for (Node c : DataLoader.customers) {
            int count = served.getOrDefault(c.id, 0);
            if (count == 0) {
                out.add("Eq.8: customer " + c.id + " not served");
            } else if (count > 1) {
                out.add("Eq.8: customer " + c.id + " served " + count + " times");
            }
        }

        // ======================================================
        // Eq.(2): depot EV balance
        // ======================================================
        Map<Integer, Integer> depotOut = new HashMap<>();
        Map<Integer, Integer> depotIn = new HashMap<>();

        for (Node d : DataLoader.depots) {
            depotOut.put(d.id, 0);
            depotIn.put(d.id, 0);
        }

        for (EVRoute r : sol.evRoutes) {
            depotOut.merge(r.startDepotId, 1, Integer::sum);
            depotIn.merge(r.endDepotId, 1, Integer::sum);
        }

        for (Node d : DataLoader.depots) {
            int outCount = depotOut.getOrDefault(d.id, 0);
            int inCount = depotIn.getOrDefault(d.id, 0);

            if (outCount != inCount) {
                out.add("Eq.2: depot " + d.id
                        + " EV balance violated, out=" + outCount
                        + ", in=" + inCount);
            }
        }

        // ======================================================
        // EV route constraints
        // ======================================================
        for (EVRoute route : sol.evRoutes) {
            double totalDemand = route.totalDemandServedByEVAndDrone();

            if (totalDemand > Constants.EV_CAPACITY + 1e-9) {
                out.add(String.format(Locale.US,
                        "Eq.9: EV%d demand %.3f > %.3f",
                        route.evId,
                        totalDemand,
                        Constants.EV_CAPACITY));
            }

            double maxEnergyUse = Constants.EV_BATTERY - Constants.EV_MIN_ENERGY;
            if (route.energyUsed > maxEnergyUse + 1e-9) {
                out.add(String.format(Locale.US,
                        "Eq.10: EV%d energy %.5f > %.5f",
                        route.evId,
                        route.energyUsed,
                        maxEnergyUse));
            }

            if (!route.departureTimes.isEmpty()) {
                double departDepot = route.departureTimes.get(0);
                if (departDepot < Constants.T_START - 1e-9) {
                    out.add(String.format(Locale.US,
                            "Eq.6: EV%d depart %.3f < %.3f",
                            route.evId,
                            departDepot,
                            Constants.T_START));
                }
            }

            if (!route.arrivalTimes.isEmpty()) {
                double returnTime = route.arrivalTimes.get(route.arrivalTimes.size() - 1);
                if (returnTime > Constants.T_END + 1e-9) {
                    out.add(String.format(Locale.US,
                            "Eq.7: EV%d return %.3f > %.3f",
                            route.evId,
                            returnTime,
                            Constants.T_END));
                }
            }

            Map<Integer, Integer> launchCount = launchCountByNode(route);
            Map<Integer, Integer> retrieveCount = retrieveCountByNode(route);

            for (Map.Entry<Integer, Integer> e : launchCount.entrySet()) {
                Node node = DataLoader.getNode(e.getKey());
                if (!node.isDepot && e.getValue() > 2) {
                    out.add("Eq.29: node " + e.getKey()
                            + " launches " + e.getValue()
                            + " drones > 2");
                }
            }

            for (Map.Entry<Integer, Integer> e : retrieveCount.entrySet()) {
                Node node = DataLoader.getNode(e.getKey());
                if (!node.isDepot && e.getValue() > 1) {
                    out.add("Eq.27: node " + e.getKey()
                            + " retrieves " + e.getValue()
                            + " drones > 1");
                }
            }

            if (!routeDroneCarryFeasible(route)) {
                out.add("Eq.15-17: EV" + route.evId
                        + " carries more than one drone or launches without an onboard drone");
            }
        }

        // ======================================================
        // Drone constraints
        // ======================================================
        for (DroneTrip dt : sol.allDroneTrips) {
            Node servedNode = DataLoader.getCustomer(dt.serveNodeId);

            if (servedNode.demand > Constants.DRONE_CAPACITY + 1e-9) {
                out.add(String.format(Locale.US,
                        "Eq.35: drone trip Drone%d.%d serves C%d demand %.3f > %.3f",
                        dt.droneId,
                        dt.tripIndex,
                        dt.serveNodeId,
                        servedNode.demand,
                        Constants.DRONE_CAPACITY));
            }

            double remain = Constants.DRONE_BATTERY - dt.energyUsed;
            if (remain < Constants.DRONE_MIN_ENERGY - 1e-9) {
                out.add(String.format(Locale.US,
                        "Eq.18: Drone%d.%d remaining %.5f < %.5f",
                        dt.droneId,
                        dt.tripIndex,
                        remain,
                        Constants.DRONE_MIN_ENERGY));
            }

            if (dt.departTime < Constants.T_START - 1e-9) {
                out.add(String.format(Locale.US,
                        "Eq.24: Drone%d.%d depart %.3f < %.3f",
                        dt.droneId,
                        dt.tripIndex,
                        dt.departTime,
                        Constants.T_START));
            }

            if (dt.arriveTime > Constants.T_END + 1e-9) {
                out.add(String.format(Locale.US,
                        "Eq.25: Drone%d.%d arrive %.3f > %.3f",
                        dt.droneId,
                        dt.tripIndex,
                        dt.arriveTime,
                        Constants.T_END));
            }

            if (dt.launchEVId > 0) {
                EVRoute launchRoute = findRoute(sol, dt.launchEVId);
                if (launchRoute == null) {
                    out.add("Eq.37/39: launch EV" + dt.launchEVId + " not found");
                } else if (!launchRoute.visitsNode(dt.launchNodeId)) {
                    out.add("Eq.37/39: launch node " + dt.launchNodeId
                            + " not visited by EV" + dt.launchEVId);
                }
            }

            if (dt.retrieveEVId > 0) {
                EVRoute retrieveRoute = findRoute(sol, dt.retrieveEVId);
                if (retrieveRoute == null) {
                    out.add("Eq.19-21: retrieve EV" + dt.retrieveEVId + " not found");
                } else if (!retrieveRoute.visitsNode(dt.retrieveNodeId)) {
                    out.add("Eq.19-21: retrieve node " + dt.retrieveNodeId
                            + " not visited by EV" + dt.retrieveEVId);
                } else {
                    double evDepartureAtRetrieve = retrieveRoute.getDepartureAtNode(dt.retrieveNodeId);
                    if (dt.arriveTime > evDepartureAtRetrieve + 1e-9) {
                        out.add(String.format(Locale.US,
                                "Eq.26: Drone%d.%d arrive %.3f > EV%d depart %.3f at node %d",
                                dt.droneId,
                                dt.tripIndex,
                                dt.arriveTime,
                                dt.retrieveEVId,
                                evDepartureAtRetrieve,
                                dt.retrieveNodeId));
                    }
                }
            }
        }

        // ======================================================
        // Eq.(41): drone continuity
        // ======================================================
        Map<Integer, List<DroneTrip>> byDrone = tripsByDrone(sol);
        for (Map.Entry<Integer, List<DroneTrip>> entry : byDrone.entrySet()) {
            int droneId = entry.getKey();
            List<DroneTrip> trips = entry.getValue();
            trips.sort(Comparator.comparingInt(t -> t.tripIndex));

            for (int i = 1; i < trips.size(); i++) {
                DroneTrip prev = trips.get(i - 1);
                DroneTrip next = trips.get(i);

                if (next.departTime + 1e-9 < prev.arriveTime) {
                    out.add(String.format(Locale.US,
                            "Eq.41: Drone%d trip%d departs %.3f before trip%d arrives %.3f",
                            droneId,
                            next.tripIndex,
                            next.departTime,
                            prev.tripIndex,
                            prev.arriveTime));
                }

                if (next.launchNodeId != prev.retrieveNodeId) {
                    out.add("Eq.41: Drone" + droneId
                            + " trip" + next.tripIndex
                            + " launches at node " + next.launchNodeId
                            + " but previous trip retrieved at node "
                            + prev.retrieveNodeId);
                }

                if (prev.retrieveEVId > 0 && next.launchEVId != prev.retrieveEVId) {
                    out.add("Eq.41: Drone" + droneId
                            + " owner continuity violated: previous retrieve EV"
                            + prev.retrieveEVId
                            + ", next launch EV" + next.launchEVId);
                }
            }
        }

        return out;
    }

    // ==========================================================
    // Helpers
    // ==========================================================

    public static EVRoute findRoute(Solution sol, int evId) {
        for (EVRoute r : sol.evRoutes) {
            if (r.evId == evId) return r;
        }
        return null;
    }

    private static Map<Integer, Integer> servedCustomerCount(Solution sol) {
        Map<Integer, Integer> served = new HashMap<>();

        for (Node c : DataLoader.customers) {
            served.put(c.id, 0);
        }

        for (EVRoute r : sol.evRoutes) {
            for (int cid : r.customerIds) {
                served.merge(cid, 1, Integer::sum);
            }
        }

        for (DroneTrip dt : sol.allDroneTrips) {
            served.merge(dt.serveNodeId, 1, Integer::sum);
        }

        return served;
    }

    private static Map<Integer, Integer> launchCountByNode(EVRoute route) {
        Map<Integer, Integer> count = new HashMap<>();

        for (DroneTrip dt : route.droneTrips) {
            if (dt.launchEVId == route.evId) {
                count.merge(dt.launchNodeId, 1, Integer::sum);
            }
        }

        return count;
    }

    private static Map<Integer, Integer> retrieveCountByNode(EVRoute route) {
        Map<Integer, Integer> count = new HashMap<>();

        for (DroneTrip dt : route.droneTrips) {
            if (dt.retrieveEVId == route.evId) {
                count.merge(dt.retrieveNodeId, 1, Integer::sum);
            }
        }

        return count;
    }

    private static boolean routeDroneCarryFeasible(EVRoute route) {
        int onboard = 1;

        for (int nodeId : route.customerIds) {
            int launches = route.launchCountAtNode(nodeId);
            int retrieves = route.retrieveCountAtNode(nodeId);

            Node node = DataLoader.getNode(nodeId);
            if (!node.isDepot) {
                if (launches > 2 || retrieves > 1) {
                    return false;
                }
                if (launches == 2 && retrieves != 1) {
                    return false;
                }
            }

            Integer next = feasibleOnboardAfterNode(onboard, launches, retrieves);
            if (next == null) {
                return false;
            }
            onboard = next;
        }

        return onboard >= 0 && onboard <= 1;
    }

    private static Integer feasibleOnboardAfterNode(int onboardBefore, int launches, int retrieves) {
        return feasibleOnboardDfs(onboardBefore, launches, retrieves, new HashSet<>());
    }

    private static Integer feasibleOnboardDfs(int onboard, int launchesLeft, int retrievesLeft, Set<String> seen) {
        if (onboard < 0 || onboard > 1) {
            return null;
        }
        if (launchesLeft == 0 && retrievesLeft == 0) {
            return onboard;
        }

        String key = onboard + "/" + launchesLeft + "/" + retrievesLeft;
        if (!seen.add(key)) {
            return null;
        }

        if (launchesLeft > 0 && onboard > 0) {
            Integer afterLaunch = feasibleOnboardDfs(onboard - 1, launchesLeft - 1, retrievesLeft, seen);
            if (afterLaunch != null) {
                return afterLaunch;
            }
        }

        if (retrievesLeft > 0 && onboard < 1) {
            Integer afterRetrieve = feasibleOnboardDfs(onboard + 1, launchesLeft, retrievesLeft - 1, seen);
            if (afterRetrieve != null) {
                return afterRetrieve;
            }
        }

        return null;
    }

    private static Map<Integer, List<DroneTrip>> tripsByDrone(Solution sol) {
        Map<Integer, List<DroneTrip>> byDrone = new HashMap<>();

        for (DroneTrip dt : sol.allDroneTrips) {
            byDrone.computeIfAbsent(dt.droneId, k -> new ArrayList<>()).add(dt);
        }

        return byDrone;
    }
}