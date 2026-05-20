package algorithm;

import model.DroneTrip;
import model.EVRoute;
import model.Node;
import model.Solution;
import util.DataLoader;
import util.EnergyUtil;
import util.TimeUtil;

import java.util.*;
import java.util.stream.Collectors;

public class Decoder {

    private Decoder() {
    }

    public static Solution decode(List<Integer> chromosome) {
        Solution sol = new Solution();

        // ===== Stage 1: Depot-centric clustering & depot-drone assignment =====
        Set<Integer> servedByDepotDrone = assignDepotDrones(sol, chromosome);

        // ===== Stage 2: Global EV route construction =====
        List<Integer> remainingCustomers = new ArrayList<>();

        for (int cid : chromosome) {
            if (!servedByDepotDrone.contains(cid)) {
                remainingCustomers.add(cid);
            }
        }

        sol.evRoutes.addAll(buildGlobalEVRoutes(remainingCustomers));

        pruneEmptyRoutes(sol);
        assignOpenEndDepots(sol);
        refreshAllRoutes(sol);

        // ===== Stage 3: Collaborative Drone-EV Integration =====
        // Hàm này đã tự prune/open-depot/refresh/computeCost/check feasible ở cuối.
        assignCollaborativeDroneTrips(sol);

        return sol;
    }
    // ==========================================================
    // Stage 1: depot drones
    // ==========================================================
    private static Set<Integer> assignDepotDrones(Solution sol, List<Integer> chromosome) {
        Set<Integer> served = new HashSet<>();

        Map<Integer, List<Integer>> clusters = new LinkedHashMap<>();
        for (Node depot : DataLoader.depots) {
            clusters.put(depot.id, new ArrayList<>());
        }

        for (int cid : chromosome) {
            Node c = DataLoader.getCustomer(cid);
            Node d = nearestDepot(c);
            clusters.get(d.id).add(cid);
        }

        int nextDroneId = 1;

        /*
         * Mỗi depot có danh sách drone đang ở depot đó.
         * Drone sau khi bay Depot -> Customer -> Depot có thể tiếp tục được dispatch
         * với tripIndex + 1, đúng tinh thần multi-trip depot-drone trong paper.
         */
        Map<Integer, List<DepotDroneState>> depotDronePool = new HashMap<>();
        for (Node depot : DataLoader.depots) {
            depotDronePool.put(depot.id, new ArrayList<>());
        }

        for (Node depot : DataLoader.depots) {
            List<Integer> candidates = clusters.get(depot.id).stream()
                    .filter(DataLoader::isDroneEligibleCustomer)
                    .sorted(Comparator.comparingDouble(cid ->
                            DataLoader.distance(depot.id, cid)))
                    .collect(Collectors.toList());

            for (int cid : candidates) {
                if (served.contains(cid)) continue;

                double d = DataLoader.distance(depot.id, cid);

                if (!EnergyUtil.droneHasEnoughEnergy(d, d)) {
                    continue;
                }

                double duration = TimeUtil.droneTravelTime(2.0 * d)
                        + 4.0 * Constants.DRONE_T1;

                DepotDroneState bestState = null;
                double bestDepart = Double.POSITIVE_INFINITY;
                double bestArrive = Double.POSITIVE_INFINITY;

                /*
                 * Ưu tiên reuse drone đã quay về depot.
                 * Nếu nhiều drone reuse được, chọn drone có thời điểm hoàn thành sớm nhất.
                 */
                for (DepotDroneState state : depotDronePool.get(depot.id)) {
                    double depart = Math.max(
                            Constants.T_START,
                            state.availableTime + Constants.DRONE_T1 + Constants.DRONE_T2
                    );

                    double arrive = depart + duration;

                    if (arrive > Constants.T_END + 1e-9) {
                        continue;
                    }

                    if (arrive < bestArrive) {
                        bestArrive = arrive;
                        bestDepart = depart;
                        bestState = state;
                    }
                }

                /*
                 * Nếu chưa có drone nào ở depot có thể reuse,
                 * tạo drone mới tại depot.
                 */
                if (bestState == null) {
                    double depart = Constants.T_START
                            + Constants.DRONE_T2
                            + Constants.DRONE_T1;

                    double arrive = depart + duration;

                    if (arrive > Constants.T_END + 1e-9) {
                        continue;
                    }

                    bestState = new DepotDroneState(
                            nextDroneId++,
                            depot.id,
                            Constants.T_START,
                            1
                    );

                    depotDronePool.get(depot.id).add(bestState);

                    bestDepart = depart;
                    bestArrive = arrive;
                }

                DroneTrip dt = new DroneTrip(
                        bestState.droneId,
                        bestState.nextTripIndex,
                        depot.id,
                        cid,
                        depot.id,
                        -1,
                        -1
                );

                dt.departTime = bestDepart;
                dt.arriveTime = bestArrive;
                dt.hoverTime = 0.0;
                dt.energyUsed = EnergyUtil.droneEnergy(d, d, 0.0);

                sol.allDroneTrips.add(dt);
                served.add(cid);

                bestState.availableTime = bestArrive;
                bestState.nextTripIndex++;
            }
        }

        return served;
    }

    // ==========================================================
    // Stage 2: global EV route construction
    // ==========================================================
    private static List<EVRoute> buildGlobalEVRoutes(List<Integer> customers) {
        List<EVRoute> routes = new ArrayList<>();
        if (customers.isEmpty()) return routes;

        int nextEvId = 1;

        for (int cid : customers) {
            Insertion best = findBestInsertion(routes, cid);

            if (best != null) {
                best.route.customerIds.add(best.position, cid);
                rebuildRoute(best.route, best.route.startDepotId, best.route.startDepotId);
            } else {
                int depotId = bestStartDepotForCustomer(cid);

                EVRoute route = new EVRoute(nextEvId++, depotId);
                route.customerIds.add(cid);
                rebuildRoute(route, route.startDepotId, route.startDepotId);

                routes.add(route);
            }
        }

        return routes;
    }

    private static Insertion findBestInsertion(List<EVRoute> routes, int cid) {
        Insertion best = null;

        for (EVRoute route : routes) {
            List<Integer> positions = candidateInsertionPositions(route, cid);

            for (int pos : positions) {
                EVRoute test = new EVRoute(route);
                test.customerIds.add(pos, cid);
                rebuildRoute(test, test.startDepotId, test.startDepotId);

                if (!routeLevelFeasible(test)) continue;

                double delta = test.energyUsed - route.energyUsed;

                if (best == null || delta < best.delta) {
                    best = new Insertion();
                    best.route = route;
                    best.position = pos;
                    best.delta = delta;
                }
            }
        }

        return best;
    }

    private static List<Integer> candidateInsertionPositions(EVRoute route, int cid) {
        Set<Integer> positions = new LinkedHashSet<>();

        int size = route.customerIds.size();

        // Luôn xét đầu và cuối route.
        positions.add(0);
        positions.add(size);

        /*
         * Xét thêm vị trí trước/sau một vài customer gần cid nhất.
         * Đây là bounded insertion, không thử toàn bộ route.
         */
        List<Integer> orderedIndexes = new ArrayList<>();
        for (int i = 0; i < size; i++) {
            orderedIndexes.add(i);
        }

        orderedIndexes.sort(Comparator.comparingDouble(
                i -> DataLoader.distance(cid, route.customerIds.get(i))
        ));

        int limit = Math.min(
                Constants.STAGE2_INSERT_NEAREST_POSITIONS,
                orderedIndexes.size()
        );

        for (int k = 0; k < limit; k++) {
            int idx = orderedIndexes.get(k);

            positions.add(idx);
            positions.add(idx + 1);
        }

        return new ArrayList<>(positions);
    }
    // ==========================================================
    // Route improvement
    // ==========================================================

    private static boolean routeLevelFeasible(EVRoute route) {
        if (route.totalDemandServedByEVAndDrone() > Constants.EV_CAPACITY + 1e-9) {
            return false;
        }

        double maxEnergyUse = Constants.EV_BATTERY - Constants.EV_MIN_ENERGY;
        if (route.energyUsed > maxEnergyUse + 1e-9) {
            return false;
        }

        if (route.departureTimes.isEmpty() || route.arrivalTimes.isEmpty()) {
            return false;
        }

        double departDepot = route.departureTimes.get(0);
        if (departDepot < Constants.T_START - 1e-9) {
            return false;
        }

        double returnDepot = route.arrivalTimes.get(route.arrivalTimes.size() - 1);
        if (returnDepot > Constants.T_END + 1e-9) {
            return false;
        }

        return true;
    }

    // ==========================================================
    // Open depot end assignment
    // ==========================================================
    private static void assignOpenEndDepots(Solution sol) {
        if (sol.evRoutes.isEmpty()) return;

        Map<Integer, Integer> quota = new LinkedHashMap<>();
        for (Node depot : DataLoader.depots) {
            quota.put(depot.id, 0);
        }

        for (EVRoute route : sol.evRoutes) {
            quota.merge(route.startDepotId, 1, Integer::sum);
        }

        List<EVRoute> routes = new ArrayList<>(sol.evRoutes);
        routes.sort(Comparator.comparingInt(r -> r.evId));

        List<Node> depots = new ArrayList<>(DataLoader.depots);
        depots.sort(Comparator.comparingInt(d -> d.id));

        int rSize = routes.size();
        int dSize = depots.size();

        /*
         * Precompute candidate energy:
         * energyMatrix[i][j] = EV route i quay về depot j có feasible không, năng lượng bao nhiêu.
         *
         * Nhờ vậy searchBestEndDepotAssignmentFast không phải rebuildRoute lặp lại.
         */
        double[][] energyMatrix = new double[rSize][dSize];
        boolean[][] feasibleMatrix = new boolean[rSize][dSize];

        for (int i = 0; i < rSize; i++) {
            EVRoute route = routes.get(i);

            for (int j = 0; j < dSize; j++) {
                int depotId = depots.get(j).id;

                EVRoute test = new EVRoute(route);
                rebuildRoute(test, test.startDepotId, depotId);

                if (routeLevelFeasible(test)) {
                    feasibleMatrix[i][j] = true;
                    energyMatrix[i][j] = test.energyUsed;
                } else {
                    feasibleMatrix[i][j] = false;
                    energyMatrix[i][j] = Double.POSITIVE_INFINITY;
                }
            }
        }

        OpenDepotAssignment best = new OpenDepotAssignment();
        best.totalEnergy = Double.POSITIVE_INFINITY;
        best.endDepots = new int[rSize];

        Map<Integer, Integer> used = new HashMap<>();
        for (Node depot : depots) {
            used.put(depot.id, 0);
        }

        int[] currentEndDepots = new int[rSize];

        searchBestEndDepotAssignmentFast(
                routes,
                depots,
                0,
                quota,
                used,
                currentEndDepots,
                0.0,
                best,
                feasibleMatrix,
                energyMatrix
        );

        if (Double.isInfinite(best.totalEnergy)) {
            /*
             * Fallback an toàn.
             * Nếu không tìm được tổ hợp open depot feasible, quay về depot xuất phát.
             */
            for (EVRoute route : sol.evRoutes) {
                rebuildRoute(route, route.startDepotId, route.startDepotId);
            }
            return;
        }

        /*
         * Apply assignment tốt nhất.
         */
        for (int i = 0; i < routes.size(); i++) {
            EVRoute route = routes.get(i);
            int endDepotId = best.endDepots[i];
            rebuildRoute(route, route.startDepotId, endDepotId);
        }
    }
    private static void searchBestEndDepotAssignmentFast(List<EVRoute> routes,
                                                         List<Node> depots,
                                                         int index,
                                                         Map<Integer, Integer> quota,
                                                         Map<Integer, Integer> used,
                                                         int[] currentEndDepots,
                                                         double currentEnergy,
                                                         OpenDepotAssignment best,
                                                         boolean[][] feasibleMatrix,
                                                         double[][] energyMatrix) {
        if (currentEnergy >= best.totalEnergy - 1e-9) {
            return;
        }

        if (index >= routes.size()) {
            for (Map.Entry<Integer, Integer> e : quota.entrySet()) {
                int depotId = e.getKey();
                int required = e.getValue();
                int actual = used.getOrDefault(depotId, 0);

                if (actual != required) {
                    return;
                }
            }

            best.totalEnergy = currentEnergy;
            best.endDepots = Arrays.copyOf(currentEndDepots, currentEndDepots.length);
            return;
        }

        for (int depotIndex = 0; depotIndex < depots.size(); depotIndex++) {
            Node depot = depots.get(depotIndex);
            int depotId = depot.id;

            if (!feasibleMatrix[index][depotIndex]) {
                continue;
            }

            if (used.getOrDefault(depotId, 0) >= quota.getOrDefault(depotId, 0)) {
                continue;
            }

            used.merge(depotId, 1, Integer::sum);
            currentEndDepots[index] = depotId;

            searchBestEndDepotAssignmentFast(
                    routes,
                    depots,
                    index + 1,
                    quota,
                    used,
                    currentEndDepots,
                    currentEnergy + energyMatrix[index][depotIndex],
                    best,
                    feasibleMatrix,
                    energyMatrix
            );

            used.merge(depotId, -1, Integer::sum);
        }
    }

    // ==========================================================
    // Stage 3: collaborative drone-EV integration
    // ==========================================================
    private static void assignCollaborativeDroneTrips(Solution sol) {
        refreshAllRoutes(sol);

        Set<Integer> alreadyDroneServed = sol.allDroneTrips.stream()
                .map(dt -> dt.serveNodeId)
                .collect(Collectors.toSet());

        List<DroneState> droneStates = initDroneStates(sol);

        // ===== Scenario f in paper: Depot -> Customer -> EV =====
        improveDepotDroneRetrieveByEV(sol, droneStates);

        refreshAllRoutes(sol);
        computeCost(sol);
        sol.totalPenalty = ConstraintChecker.penalty(sol);
        sol.feasible = ConstraintChecker.isFeasible(sol);

        boolean changed = true;
        int guard = 0;

        while (changed && guard < Constants.STAGE3_MAX_ROUNDS) {
            changed = false;
            guard++;

            refreshAllRoutes(sol);

            /*
             * KHÔNG duyệt trực tiếp sol.evRoutes bằng for-each.
             * Vì trong launchBestDroneFromEVNode có thể rollback solution,
             * làm sol.evRoutes bị clear/addAll và gây ConcurrentModificationException.
             */
            List<Integer> routeIds = sol.evRoutes.stream()
                    .map(r -> r.evId)
                    .collect(Collectors.toList());

            for (int routePos = 0; routePos < routeIds.size(); routePos++) {
                int evId = routeIds.get(routePos);

                EVRoute evRoute = findRouteByEvId(sol, evId);
                if (evRoute == null) {
                    continue;
                }

                int nodeIndex = 0;

                /*
                 * Dùng while thay vì for cố định size,
                 * vì khi drone move thành công, customerIds của một route có thể thay đổi.
                 */
                while (true) {
                    evRoute = findRouteByEvId(sol, evId);
                    if (evRoute == null) {
                        break;
                    }

                    if (nodeIndex >= evRoute.customerIds.size()) {
                        break;
                    }

                    int currentNodeId = evRoute.customerIds.get(nodeIndex);

                    /*
                     * Paper: EV là mobile depot cho drone.
                     * Đồng bộ các drone đang được EV này carry tại node hiện tại.
                     */
                    syncCarriedDronesWithEVAtNode(
                            droneStates,
                            evRoute,
                            currentNodeId
                    );

                    boolean launchLoop = true;

                    while (launchLoop) {
                        evRoute = findRouteByEvId(sol, evId);
                        if (evRoute == null) {
                            launchLoop = false;
                            break;
                        }

                        launchLoop = launchBestDroneFromEVNode(
                                sol,
                                droneStates,
                                evRoute,
                                currentNodeId,
                                nodeIndex,
                                alreadyDroneServed
                        );

                        if (launchLoop) {
                            changed = true;

                            /*
                             * Sau khi apply thành công, route/customer/timing có thể đổi.
                             * Refresh đúng một lần.
                             */
                            refreshAllRoutes(sol);

                            evRoute = findRouteByEvId(sol, evId);
                            if (evRoute != null && evRoute.customerIds.contains(currentNodeId)) {
                                syncCarriedDronesWithEVAtNode(
                                        droneStates,
                                        evRoute,
                                        currentNodeId
                                );
                            }
                        }
                    }

                    nodeIndex++;
                }
            }
        }

        pruneEmptyRoutes(sol);
        assignOpenEndDepots(sol);
        refreshAllRoutes(sol);

        computeCost(sol);
        sol.totalPenalty = ConstraintChecker.penalty(sol);
        sol.feasible = ConstraintChecker.isFeasible(sol);
    }

    private static List<DroneState> initDroneStates(Solution sol) {
        List<DroneState> states = new ArrayList<>();

        Map<Integer, DroneTrip> lastTripByDrone = new HashMap<>();

        for (DroneTrip dt : sol.allDroneTrips) {
            DroneTrip last = lastTripByDrone.get(dt.droneId);
            if (last == null || dt.tripIndex > last.tripIndex) {
                lastTripByDrone.put(dt.droneId, dt);
            }
        }

        for (DroneTrip last : lastTripByDrone.values()) {
            states.add(new DroneState(
                    last.droneId,
                    last.retrieveNodeId,
                    last.retrieveEVId,
                    last.arriveTime,
                    last.tripIndex + 1
            ));
        }

        int nextDroneId = sol.allDroneTrips.stream()
                .mapToInt(dt -> dt.droneId)
                .max()
                .orElse(0) + 1;

        for (EVRoute route : sol.evRoutes) {
            states.add(new DroneState(
                    nextDroneId++,
                    route.startDepotId,
                    route.evId,
                    Constants.T_START,
                    1
            ));
        }

        return states;
    }


    private static boolean launchBestDroneFromEVNode(Solution sol,
                                                     List<DroneState> droneStates,
                                                     EVRoute evRoute,
                                                     int launchNodeId,
                                                     int launchIndex,
                                                     Set<Integer> alreadyDroneServed) {
        if (evRoute.launchCountAtNode(launchNodeId) >= 2) {
            return false;
        }

        /*
         * Chỉ build context khi thật sự có drone đang ở node này.
         * Nếu không có drone để launch thì không cần tạo candidate.
         */
        List<DroneState> availableStates = new ArrayList<>();

        for (DroneState state : droneStates) {
            if (state.ownerEVId != evRoute.evId) continue;
            if (state.currentNodeId != launchNodeId) continue;
            availableStates.add(state);
        }

        if (availableStates.isEmpty()) {
            return false;
        }

        Stage3SearchContext ctx = new Stage3SearchContext(sol, alreadyDroneServed);

        List<Integer> candidateServe =
                candidateDroneCustomersWholeSolution(ctx, launchNodeId);

        if (candidateServe.isEmpty()) {
            return false;
        }

        DroneMove best = null;
        DroneState bestState = null;

        for (DroneState state : availableStates) {
            DroneMove move = findBestSequentialDroneMove(
                    sol,
                    evRoute,
                    state,
                    launchNodeId,
                    launchIndex,
                    ctx,
                    candidateServe
            );

            if (move != null) {
                if (best == null || move.objectiveAfter < best.objectiveAfter) {
                    best = move;
                    bestState = state;
                }
            }
        }

        if (best == null) return false;

        return applySequentialDroneMoveSafely(
                sol,
                bestState,
                best,
                alreadyDroneServed
        );
    }
    private static DroneMove findBestSequentialDroneMove(Solution sol,
                                                         EVRoute launchRoute,
                                                         DroneState state,
                                                         int launchNodeId,
                                                         int launchIndex,
                                                         Stage3SearchContext ctx,
                                                         List<Integer> candidateServe) {
        DroneMove best = null;


        for (int serveId : candidateServe) {
            EVRoute servedRoute = ctx.routeByCustomer.get(serveId);
            if (servedRoute == null) continue;

            // g: EV -> customer -> same EV
            for (int retrieveNodeId : candidateRetrieveNodesAfterLaunch(launchRoute, launchIndex, serveId)) {
                DroneMove move = buildDroneMoveIfFeasible(
                        sol,
                        launchRoute,
                        servedRoute,
                        launchRoute,
                        state,
                        launchNodeId,
                        serveId,
                        retrieveNodeId,
                        ctx
                );

                if (move != null && evaluateMoveObjective(sol, move, ctx)) {
                    if (best == null || move.objectiveAfter < best.objectiveAfter) {
                        best = move;
                    }
                }
            }

            // h: EV -> customer -> other EV
            for (EVRoute retrieveRoute : sol.evRoutes) {
                if (retrieveRoute.evId == launchRoute.evId) continue;

                for (int retrieveNodeId : candidateRetrieveNodesOnRouteLight(retrieveRoute, serveId)) {
                    DroneMove move = buildDroneMoveIfFeasible(
                            sol,
                            launchRoute,
                            servedRoute,
                            retrieveRoute,
                            state,
                            launchNodeId,
                            serveId,
                            retrieveNodeId,
                            ctx
                    );

                    if (move != null && evaluateMoveObjective(sol, move, ctx)) {
                        if (best == null || move.objectiveAfter < best.objectiveAfter) {
                            best = move;
                        }
                    }
                }
            }

            // i: EV -> customer -> Depot
            for (Node depot : DataLoader.depots) {
                DroneMove move = buildDroneMoveIfFeasible(
                        sol,
                        launchRoute,
                        servedRoute,
                        null,
                        state,
                        launchNodeId,
                        serveId,
                        depot.id,
                        ctx
                );

                if (move != null && evaluateMoveObjective(sol, move, ctx)) {
                    if (best == null || move.objectiveAfter < best.objectiveAfter) {
                        best = move;
                    }
                }
            }
        }

        if (best == null) {
            return null;
        }

        return best;
    }

    private static List<Integer> candidateDroneCustomersWholeSolution(Stage3SearchContext ctx,
                                                                      int launchNodeId) {
        List<Integer> out = new ArrayList<>();

        for (int cid : ctx.directCustomersInRouteOrder) {
            if (ctx.droneServedCustomers.contains(cid)) continue;
            if (!DataLoader.isDroneEligibleCustomer(cid)) continue;

            // Eq.(22)-(23): node đã launch/retrieve drone
            // thì không được chuyển thành drone-served customer.
            if (ctx.droneOperationNodes.contains(cid)) continue;

            out.add(cid);
        }

        /*
         * Production candidate ranking:
         * Ưu tiên customer mà nếu bỏ khỏi EV route thì tiết kiệm EV distance nhiều,
         * đồng thời không quá xa launch node.
         *
         * Đây chỉ là ranking candidate. Feasibility và objective thật vẫn được kiểm
         * ở buildDroneMoveIfFeasible(...) và evaluateMoveObjective(...).
         */
        out.sort((a, b) -> {
            double scoreA = approximateDroneServePriority(ctx, launchNodeId, a);
            double scoreB = approximateDroneServePriority(ctx, launchNodeId, b);

            // score lớn hơn tốt hơn
            return Double.compare(scoreB, scoreA);
        });

        if (out.size() > Constants.STAGE3_SERVE_CANDIDATES) {
            return new ArrayList<>(
                    out.subList(0, Constants.STAGE3_SERVE_CANDIDATES)
            );
        }

        return out;
    }
    private static List<Integer> candidateRetrieveNodesAfterLaunch(EVRoute route,
                                                                   int launchIndex,
                                                                   int serveId) {
        if (launchIndex + 1 >= route.customerIds.size()) {
            return Collections.emptyList();
        }

        List<Integer> out = new ArrayList<>(
                route.customerIds.subList(
                        launchIndex + 1,
                        route.customerIds.size()
                )
        );

        /*
         * Retrieve node gần serve customer hơn thường tiết kiệm drone energy hơn.
         */
        out.sort(Comparator.comparingDouble(
                nodeId -> DataLoader.distance(serveId, nodeId)
        ));

        if (out.size() > Constants.STAGE3_RETRIEVE_CANDIDATES) {
            return new ArrayList<>(
                    out.subList(0, Constants.STAGE3_RETRIEVE_CANDIDATES)
            );
        }

        return out;
    }
    private static List<Integer> candidateRetrieveNodesOnRouteLight(EVRoute route,
                                                                    int serveId) {
        List<Integer> out = new ArrayList<>(route.customerIds);

        out.sort(Comparator.comparingDouble(
                nodeId -> DataLoader.distance(serveId, nodeId)
        ));

        if (out.size() > Constants.STAGE3_RETRIEVE_CANDIDATES) {
            return new ArrayList<>(
                    out.subList(0, Constants.STAGE3_RETRIEVE_CANDIDATES)
            );
        }

        return out;
    }
    private static DroneMove buildDroneMoveIfFeasible(Solution sol,
                                                      EVRoute launchRoute,
                                                      EVRoute servedRoute,
                                                      EVRoute retrieveRoute,
                                                      DroneState state,
                                                      int launchNodeId,
                                                      int serveId,
                                                      int retrieveNodeId,
                                                      Stage3SearchContext ctx) {
        if (launchNodeId == serveId) {
            return null;
        }

        if (retrieveNodeId == serveId) {
            return null;
        }

// Eq.(22)-(23): customer đã được drone serve thì không được dùng làm launch/retrieve node
        if (ctx.droneServedCustomers.contains(launchNodeId)) {
            return null;
        }

        if (ctx.droneServedCustomers.contains(retrieveNodeId)) {
            return null;
        }

        if (ctx.droneOperationNodes.contains(serveId)) {
            return null;
        }

        if (launchRoute.launchCountAtNode(launchNodeId) >= 2) {
            return null;
        }

        if (retrieveRoute != null && retrieveRoute.retrieveCountAtNode(retrieveNodeId) >= 1) {
            return null;
        }

        double d1 = DataLoader.distance(launchNodeId, serveId);
        double d2 = DataLoader.distance(serveId, retrieveNodeId);

        if (!EnergyUtil.droneHasEnoughEnergy(d1, d2)) {
            return null;
        }

        double depart = Math.max(
                state.availableTime,
                launchRoute.getDepartureAtNode(launchNodeId)
        );

        double rawArrive = depart
                + TimeUtil.droneTravelTime(d1 + d2)
                + 4.0 * Constants.DRONE_T1;

        double hover = 0.0;

        if (retrieveRoute != null) {
            double evDepartAtRetrieve = retrieveRoute.getDepartureAtNode(retrieveNodeId);

            if (rawArrive > evDepartAtRetrieve + 1e-9) {
                return null;
            }

            hover = Math.max(0.0, evDepartAtRetrieve - rawArrive);
        } else {
            if (rawArrive > Constants.T_END + 1e-9) {
                return null;
            }
        }

        double energy = EnergyUtil.droneEnergy(d1, d2, hover);

        DroneMove move = new DroneMove();
        move.launchRoute = launchRoute;
        move.servedRoute = servedRoute;
        move.retrieveRoute = retrieveRoute;

        move.launchNodeId = launchNodeId;
        move.serveNodeId = serveId;
        move.retrieveNodeId = retrieveNodeId;

        move.launchEVId = launchRoute.evId;
        move.retrieveEVId = retrieveRoute == null ? -1 : retrieveRoute.evId;

        move.departTime = depart;
        move.arriveTime = rawArrive + hover;
        move.hoverTime = hover;
        move.energyUsed = energy;
        // savingScore không còn dùng để chọn move.
// Stage 3 chọn theo objectiveAfter trong evaluateMoveObjective().
        move.savingScore = 0.0;
        move.droneId = state.droneId;
        move.tripIndex = state.nextTripIndex;
        move.objectiveAfter = Double.POSITIVE_INFINITY;

        return move;
    }

    private static void applySequentialDroneMove(Solution sol,
                                                 DroneState state,
                                                 DroneMove move,
                                                 Set<Integer> alreadyDroneServed) {
        DroneTrip dt = new DroneTrip(
                move.droneId,
                move.tripIndex,
                move.launchNodeId,
                move.serveNodeId,
                move.retrieveNodeId,
                move.launchEVId,
                move.retrieveEVId
        );

        dt.departTime = move.departTime;
        dt.arriveTime = move.arriveTime;
        dt.hoverTime = move.hoverTime;
        dt.energyUsed = move.energyUsed;

        move.servedRoute.customerIds.remove(Integer.valueOf(move.serveNodeId));

        move.launchRoute.droneTrips.add(dt);

        if (move.retrieveRoute != null && move.retrieveRoute.evId != move.launchRoute.evId) {
            move.retrieveRoute.droneTrips.add(dt);
        }

        sol.allDroneTrips.add(dt);
        alreadyDroneServed.add(move.serveNodeId);

        state.currentNodeId = move.retrieveNodeId;
        state.ownerEVId = move.retrieveEVId;
        state.availableTime = move.arriveTime;
        state.nextTripIndex++;

    }

    private static boolean applySequentialDroneMoveSafely(Solution sol,
                                                          DroneState state,
                                                          DroneMove move,
                                                          Set<Integer> alreadyDroneServed) {
        Solution before = new Solution(sol);

        int oldCurrentNodeId = state.currentNodeId;
        int oldOwnerEVId = state.ownerEVId;
        double oldAvailableTime = state.availableTime;
        int oldNextTripIndex = state.nextTripIndex;

        applySequentialDroneMove(sol, state, move, alreadyDroneServed);

        refreshAllRoutes(sol);
        computeCost(sol);
        sol.totalPenalty = ConstraintChecker.penalty(sol);
        sol.feasible = ConstraintChecker.isFeasible(sol);

        /*
         * Chỉ selected move mới check full ConstraintChecker.
         * Candidate move thì dùng fast evaluation.
         */
        if (!sol.feasible) {
            restoreSolution(sol, before);

            state.currentNodeId = oldCurrentNodeId;
            state.ownerEVId = oldOwnerEVId;
            state.availableTime = oldAvailableTime;
            state.nextTripIndex = oldNextTripIndex;

            alreadyDroneServed.remove(move.serveNodeId);
            return false;
        }

        return true;
    }
    // ==========================================================
    // Route rebuild / refresh / cost
    // ==========================================================
    private static void refreshAllRoutes(Solution sol) {
        for (EVRoute route : sol.evRoutes) {
            rebuildRoute(route, route.startDepotId, route.endDepotId);
        }
    }

    private static void rebuildRoute(EVRoute route, int startDepotId, int endDepotId) {
        Node startDepot = DataLoader.getDepot(startDepotId);
        Node endDepot = DataLoader.getDepot(endDepotId);

        route.startDepotId = startDepotId;
        route.endDepotId = endDepotId;

        route.arrivalTimes.clear();
        route.departureTimes.clear();
        route.loads.clear();
        route.departureTimeByCustomerNode.clear();

        Node pos = startDepot;
        double time = Constants.T_START;
        double remainingLoad = route.totalDemandServedByEVAndDrone();
        double energy = 0.0;

        route.arrivalTimes.add(Constants.T_START);

        double startDepart = Constants.T_START
                + route.launchCountAtNode(startDepotId) * (Constants.DRONE_T1 + Constants.DRONE_T2);

        route.departureTimes.add(startDepart);
        time = startDepart;

        for (int cid : route.customerIds) {
            Node c = DataLoader.getCustomer(cid);

            double dist = DataLoader.distance(pos.id, cid) * Constants.DETOUR_COEFF;
            double travel = TimeUtil.travelTime(time, dist);
            double e = EnergyUtil.evEnergy(time, travel, Math.max(0.0, remainingLoad));

            double arrive = time + travel;
            double depart = arrive
                    + Constants.EV_SERVICE_TIME
                    + nodeOperationTime(route, cid);

            remainingLoad -= c.demand;
            remainingLoad -= droneDemandLaunchedAt(route, cid);

            route.arrivalTimes.add(arrive);
            route.departureTimes.add(depart);
            route.departureTimeByCustomerNode.put(cid, depart);
            route.loads.add(Math.max(0.0, remainingLoad));

            time = depart;
            energy += e;
            pos = c;
        }

        double distBack = DataLoader.distance(pos.id, endDepotId) * Constants.DETOUR_COEFF;
        double tBack = TimeUtil.travelTime(time, distBack);
        double eBack = EnergyUtil.evEnergy(time, tBack, 0.0);

        route.arrivalTimes.add(time + tBack);

        route.energyUsed = energy + eBack;
        route.totalCost = Constants.EV_DISPATCH_COST + route.energyUsed;
        route.feasible = true;
    }

    private static double nodeOperationTime(EVRoute route, int nodeId) {
        /*
         * Theo paper:
         * - Retrieve được kiểm soát bởi Eq.(26): drone phải land trước khi EV rời node.
         * - Thời gian bổ sung rõ nhất tại node là khi EV launch/relaunch drone.
         * - Nếu retrieve xong rồi launch lại ở cùng node, launchCountAtNode sẽ tính lần launch đó.
         *
         * Vì vậy không cộng retrieve riêng ở đây để tránh double-count quá bảo thủ.
         */
        int launches = route.launchCountAtNode(nodeId);

        return launches * (Constants.DRONE_T1 + Constants.DRONE_T2);
    }

    private static void computeCost(Solution sol) {
        double cost = 0.0;

        cost += sol.evRoutes.size() * Constants.EV_DISPATCH_COST;

        for (EVRoute route : sol.evRoutes) {
            cost += route.energyUsed * Constants.ELECTRICITY_PRICE;
        }

        Set<Integer> usedDroneIds = new HashSet<>();
        for (DroneTrip dt : sol.allDroneTrips) {
            cost += dt.energyUsed * Constants.ELECTRICITY_PRICE;
            usedDroneIds.add(dt.droneId);
        }

        cost += usedDroneIds.size() * Constants.DRONE_DISPATCH_COST;

        sol.totalCost = cost;
    }

    private static void pruneEmptyRoutes(Solution sol) {
        sol.evRoutes.removeIf(EVRoute::isRemovableEmptyRoute);
    }

    // ==========================================================
    // Helpers
    // ==========================================================
    private static Node nearestDepot(Node c) {
        Node best = null;
        double bestD = Double.MAX_VALUE;

        for (Node d : DataLoader.depots) {
            double dist = DataLoader.distance(c.id, d.id);
            if (dist < bestD) {
                bestD = dist;
                best = d;
            }
        }

        return best;
    }

    private static int bestStartDepotForCustomer(int cid) {
        Node c = DataLoader.getCustomer(cid);

        Node best = null;
        double bestDist = Double.MAX_VALUE;

        for (Node d : DataLoader.depots) {
            double dist = DataLoader.distance(d.id, cid);
            if (dist < bestDist) {
                bestDist = dist;
                best = d;
            }
        }

        return best.id;
    }

    private static void removeRouteByEvId(Solution sol, int evId) {
        sol.evRoutes.removeIf(r -> r.evId == evId);
    }

    private static void normalizeEvIds(Solution sol) {
        sol.evRoutes.sort(Comparator.comparingInt(r -> r.evId));

        int id = 1;
        for (EVRoute route : sol.evRoutes) {
            int oldId = route.evId;
            route.evId = id;

            for (DroneTrip dt : route.droneTrips) {
                if (dt.launchEVId == oldId) {
                    dt.launchEVId = id;
                }
                if (dt.retrieveEVId == oldId) {
                    dt.retrieveEVId = id;
                }
            }

            id++;
        }
    }
    private static void syncCarriedDronesWithEVAtNode(List<DroneState> droneStates,
                                                      EVRoute evRoute,
                                                      int nodeId) {
        double evDepart = evRoute.getDepartureAtNode(nodeId);

        for (DroneState state : droneStates) {
            /*
             * Chỉ đồng bộ drone với EV nếu:
             * 1. Drone đang thuộc EV này
             * 2. Drone đã thật sự available trước hoặc tại thời điểm EV rời node hiện tại
             *
             * Nếu state.availableTime > evDepart, nghĩa là drone chỉ được retrieve
             * ở một node tương lai, không được kéo nó quay ngược về node hiện tại.
             */
            if (state.ownerEVId == evRoute.evId
                    && state.availableTime <= evDepart + 1e-9) {
                state.currentNodeId = nodeId;
                state.availableTime = Math.max(state.availableTime, evDepart);
            }
        }
    }

    private static EVRoute findRouteByEvId(Solution sol, int evId) {
        for (EVRoute route : sol.evRoutes) {
            if (route.evId == evId) {
                return route;
            }
        }
        return null;
    }

    private static boolean evaluateMoveObjective(Solution sol,
                                                 DroneMove move,
                                                 Stage3SearchContext ctx) {
        EVRoute launchRoute = move.launchRoute;
        EVRoute servedRoute = move.servedRoute;
        EVRoute retrieveRoute = move.retrieveRoute;

        if (launchRoute == null || servedRoute == null) {
            return false;
        }

        if (move.retrieveEVId > 0 && retrieveRoute == null) {
            return false;
        }

        /*
         * Chỉ copy các route bị ảnh hưởng:
         * - route launch drone
         * - route đang chứa customer chuyển sang drone
         * - route retrieve drone nếu retrieve bởi EV khác
         */
        Map<Integer, EVRoute> affected = new HashMap<>();

        EVRoute launchCopy = affectedCopy(affected, launchRoute);
        EVRoute servedCopy = affectedCopy(affected, servedRoute);

        EVRoute retrieveCopy = null;
        if (retrieveRoute != null) {
            retrieveCopy = affectedCopy(affected, retrieveRoute);
        }

        DroneTrip dt = new DroneTrip(
                move.droneId,
                move.tripIndex,
                move.launchNodeId,
                move.serveNodeId,
                move.retrieveNodeId,
                move.launchEVId,
                move.retrieveEVId
        );

        dt.departTime = move.departTime;
        dt.arriveTime = move.arriveTime;
        dt.hoverTime = move.hoverTime;
        dt.energyUsed = move.energyUsed;

        servedCopy.customerIds.remove(Integer.valueOf(move.serveNodeId));

        launchCopy.droneTrips.add(dt);

        if (retrieveCopy != null && retrieveCopy.evId != launchCopy.evId) {
            retrieveCopy.droneTrips.add(dt);
        }

        /*
         * Rebuild chỉ route bị ảnh hưởng.
         */
        for (EVRoute route : affected.values()) {
            rebuildRoute(route, route.startDepotId, route.endDepotId);

            if (!route.isRemovableEmptyRoute() && !routeLevelFeasible(route)) {
                return false;
            }

            if (!nodeOperationFeasible(route)) {
                return false;
            }
        }

        /*
         * Check nhanh drone move.
         * Các check chính đã nằm trong buildDroneMoveIfFeasible:
         * - payload
         * - energy
         * - time sync
         * - launch/retrieve node validity
         */
        if (!singleDroneTripBasicFeasible(dt)) {
            return false;
        }

        double objective = sol.totalCost;

        /*
         * Trừ cost cũ của các route bị ảnh hưởng,
         * cộng cost mới của route copy.
         */
        for (Map.Entry<Integer, EVRoute> entry : affected.entrySet()) {
            EVRoute original = ctx.routeByEvId.get(entry.getKey());
            EVRoute changed = entry.getValue();

            if (original == null) {
                return false;
            }

            objective -= Constants.EV_DISPATCH_COST;
            objective -= original.energyUsed * Constants.ELECTRICITY_PRICE;

            if (!changed.isRemovableEmptyRoute()) {
                objective += Constants.EV_DISPATCH_COST;
                objective += changed.energyUsed * Constants.ELECTRICITY_PRICE;
            }
        }

        /*
         * Thêm energy drone trip mới.
         */
        objective += move.energyUsed * Constants.ELECTRICITY_PRICE;

        /*
         * Thêm drone dispatch cost nếu đây là droneId mới.
         * Nếu drone đã có trip trước đó, không cộng lại.
         */
        if (!ctx.usedDroneIds.contains(move.droneId)) {
            objective += Constants.DRONE_DISPATCH_COST;
        }

        move.objectiveAfter = objective;
        return true;
    }
    private static void improveDepotDroneRetrieveByEV(Solution sol,
                                                      List<DroneState> droneStates) {
        refreshAllRoutes(sol);

        /*
         * Cache customer đã được drone phục vụ.
         * Hàm này chỉ đổi retrieve node của một depot-drone trip,
         * không đổi serveNodeId, nên set này ổn định trong toàn hàm.
         */
        Set<Integer> droneServedCustomers = new HashSet<>();
        for (DroneTrip dt : sol.allDroneTrips) {
            droneServedCustomers.add(dt.serveNodeId);
        }

        /*
         * Duyệt snapshot allDroneTrips để an toàn khi update trip thật.
         */
        for (DroneTrip trip : new ArrayList<>(sol.allDroneTrips)) {

            // Chỉ xét trip dạng Depot -> Customer -> Depot
            if (trip.launchEVId > 0) continue;
            if (trip.retrieveEVId > 0) continue;

            /*
             * Depot -> serve customer distance.
             * Dùng DataLoader.distance(...) thay Node.distance(...)
             * để tận dụng ma trận khoảng cách đã cache.
             */
            double d1 = DataLoader.distance(
                    trip.launchNodeId,
                    trip.serveNodeId
            );

            DroneTrip bestCandidate = null;
            EVRoute bestRetrieveRoute = null;

            /*
             * Chỉ nhận Depot -> Customer -> EV nếu nó không làm drone trip tệ hơn
             * so với Depot -> Customer -> Depot hiện tại.
             *
             * Đây là objective-first, không ép drone về EV bằng mọi giá.
             */
            double bestEnergy = trip.energyUsed + 0.02;
            for (EVRoute route : sol.evRoutes) {
                for (int retrieveNodeId : candidateRetrieveNodesOnRouteLight(route, trip.serveNodeId)) {

                    // Eq.27: customer node chỉ retrieve tối đa 1 drone
                    if (route.retrieveCountAtNode(retrieveNodeId) >= 1) {
                        continue;
                    }

                    /*
                     * Eq.22-Eq.23:
                     * customer đã được drone serve thì không được làm launch/retrieve node.
                     */
                    if (droneServedCustomers.contains(retrieveNodeId)) {
                        continue;
                    }

                    /*
                     * serve customer -> retrieve node distance.
                     */
                    double d2 = DataLoader.distance(
                            trip.serveNodeId,
                            retrieveNodeId
                    );

                    if (!EnergyUtil.droneHasEnoughEnergy(d1, d2)) {
                        continue;
                    }

                    double depart = trip.departTime;

                    double rawArrive = depart
                            + TimeUtil.droneTravelTime(d1 + d2)
                            + 4.0 * Constants.DRONE_T1;

                    double evDepartAtRetrieve =
                            route.getDepartureAtNode(retrieveNodeId);

                    // Eq.26: drone phải land trước khi EV rời retrieve node
                    if (rawArrive > evDepartAtRetrieve + 1e-9) {
                        continue;
                    }

                    double hover = Math.max(0.0, evDepartAtRetrieve - rawArrive);
                    double energy = EnergyUtil.droneEnergy(d1, d2, hover);

                    DroneTrip candidate = new DroneTrip(trip);
                    candidate.retrieveNodeId = retrieveNodeId;
                    candidate.retrieveEVId = route.evId;
                    candidate.arriveTime = rawArrive + hover;
                    candidate.hoverTime = hover;
                    candidate.energyUsed = energy;

                    /*
                     * Giữ nguyên logic an toàn hiện tại:
                     * test trên bản copy, feasible thì mới chọn candidate.
                     * Đây là phần đúng và không nên bỏ vội.
                     */
                    /*
                     * Fast evaluation cho scenario Depot -> Customer -> EV.
                     *
                     * Các điều kiện feasibility chính đã check ở trên:
                     * - retrieve node thuộc EV route
                     * - retrieve node chưa retrieve drone khác
                     * - retrieve node không phải customer đã drone serve
                     * - drone đủ pin
                     * - drone arrive trước khi EV rời retrieve node
                     *
                     * Vì chỉ đổi retrieve node của drone, chưa đổi EV customer sequence,
                     * không cần copy full solution cho từng candidate.
                     */
                    if (energy <= trip.energyUsed + 0.02 + 1e-9
                            && energy < bestEnergy) {
                        bestEnergy = energy;
                        bestCandidate = candidate;
                        bestRetrieveRoute = route;
                    }
                }
            }

            if (bestCandidate != null && bestRetrieveRoute != null) {
                /*
                 * Snapshot để rollback nếu commit vào solution thật làm phát sinh violation.
                 */
                Solution beforeCommit = new Solution(sol);
                List<DroneState> beforeStates = copyDroneStates(droneStates);

                trip.retrieveNodeId = bestCandidate.retrieveNodeId;
                trip.retrieveEVId = bestCandidate.retrieveEVId;
                trip.arriveTime = bestCandidate.arriveTime;
                trip.hoverTime = bestCandidate.hoverTime;
                trip.energyUsed = bestCandidate.energyUsed;

                if (!bestRetrieveRoute.droneTrips.contains(trip)) {
                    bestRetrieveRoute.droneTrips.add(trip);
                }

                refreshAllRoutes(sol);
                computeCost(sol);
                sol.totalPenalty = ConstraintChecker.penalty(sol);
                sol.feasible = ConstraintChecker.isFeasible(sol);

                /*
                 * Chỉ khi solution thật vẫn feasible mới cập nhật DroneState.
                 * Nếu không feasible thì rollback.
                 */
                if (!sol.feasible) {
                    restoreSolution(sol, beforeCommit);
                    droneStates.clear();
                    droneStates.addAll(beforeStates);
                    continue;
                }

                /*
                 * Cập nhật DroneState để drone sau khi được EV retrieve
                 * có thể được EV đó carry và launch tiếp ở các node sau.
                 */
                for (DroneState state : droneStates) {
                    if (state.droneId == trip.droneId) {
                        state.currentNodeId = trip.retrieveNodeId;
                        state.ownerEVId = trip.retrieveEVId;
                        state.availableTime = trip.arriveTime;
                        state.nextTripIndex = Math.max(
                                state.nextTripIndex,
                                trip.tripIndex + 1
                        );
                    }
                }
            }
        }
    }

    private static List<DroneState> copyDroneStates(List<DroneState> source) {
        List<DroneState> copy = new ArrayList<>();

        for (DroneState s : source) {
            copy.add(new DroneState(
                    s.droneId,
                    s.currentNodeId,
                    s.ownerEVId,
                    s.availableTime,
                    s.nextTripIndex
            ));
        }

        return copy;
    }

    private static void restoreSolution(Solution target, Solution snapshot) {
        Solution copy = new Solution(snapshot);

        target.evRoutes.clear();
        target.evRoutes.addAll(copy.evRoutes);

        target.allDroneTrips.clear();
        target.allDroneTrips.addAll(copy.allDroneTrips);

        target.totalCost = copy.totalCost;
        target.totalPenalty = copy.totalPenalty;
        target.feasible = copy.feasible;
    }

    private static DroneTrip findDroneTrip(Solution sol, int droneId, int tripIndex) {
        for (DroneTrip dt : sol.allDroneTrips) {
            if (dt.droneId == droneId && dt.tripIndex == tripIndex) {
                return dt;
            }
        }
        return null;
    }

    private static double droneDemandLaunchedAt(EVRoute route, int nodeId) {
        double sum = 0.0;

        for (DroneTrip dt : route.droneTrips) {
            if (dt.launchEVId == route.evId && dt.launchNodeId == nodeId) {
                sum += DataLoader.getCustomer(dt.serveNodeId).demand;
            }
        }

        return sum;
    }

    private static EVRoute affectedCopy(Map<Integer, EVRoute> affected,
                                        EVRoute original) {
        EVRoute copy = affected.get(original.evId);

        if (copy == null) {
            copy = new EVRoute(original);
            affected.put(original.evId, copy);
        }

        return copy;
    }

    private static boolean nodeOperationFeasible(EVRoute route) {
        Set<Integer> nodes = new HashSet<>();
        nodes.addAll(route.customerIds);

        for (DroneTrip dt : route.droneTrips) {
            nodes.add(dt.launchNodeId);
            nodes.add(dt.retrieveNodeId);
        }

        for (int nodeId : nodes) {
            Node node = DataLoader.getNode(nodeId);

            if (node.isDepot) {
                continue;
            }

            if (route.launchCountAtNode(nodeId) > 2) {
                return false;
            }

            if (route.retrieveCountAtNode(nodeId) > 1) {
                return false;
            }
        }

        return true;
    }

    private static boolean singleDroneTripBasicFeasible(DroneTrip dt) {
        Node servedNode = DataLoader.getCustomer(dt.serveNodeId);

        if (servedNode.demand > Constants.DRONE_CAPACITY + 1e-9) {
            return false;
        }

        double remain = Constants.DRONE_BATTERY - dt.energyUsed;
        if (remain < Constants.DRONE_MIN_ENERGY - 1e-9) {
            return false;
        }

        if (dt.departTime < Constants.T_START - 1e-9) {
            return false;
        }

        if (dt.arriveTime > Constants.T_END + 1e-9) {
            return false;
        }

        return true;
    }

    private static double approximateDroneServePriority(Stage3SearchContext ctx,
                                                        int launchNodeId,
                                                        int serveId) {
        EVRoute route = ctx.routeByCustomer.get(serveId);
        if (route == null) {
            return Double.NEGATIVE_INFINITY;
        }

        Integer prevNodeId = ctx.prevNodeByCustomer.get(serveId);
        Integer nextNodeId = ctx.nextNodeByCustomer.get(serveId);

        if (prevNodeId == null || nextNodeId == null) {
            return Double.NEGATIVE_INFINITY;
        }

        double evSaving = DataLoader.distance(prevNodeId, serveId)
                + DataLoader.distance(serveId, nextNodeId)
                - DataLoader.distance(prevNodeId, nextNodeId);

        double launchToServe = DataLoader.distance(launchNodeId, serveId);

        return evSaving - launchToServe;
    }

    private static class Insertion {
        EVRoute route;
        int position;
        double delta;
    }

    private static class DroneState {
        int droneId;
        int currentNodeId;
        int ownerEVId;
        double availableTime;
        int nextTripIndex;

        DroneState(int droneId, int currentNodeId, int ownerEVId, double availableTime, int nextTripIndex) {
            this.droneId = droneId;
            this.currentNodeId = currentNodeId;
            this.ownerEVId = ownerEVId;
            this.availableTime = availableTime;
            this.nextTripIndex = nextTripIndex;
        }
    }

    private static class DepotDroneState {
        int droneId;
        int depotId;
        double availableTime;
        int nextTripIndex;

        DepotDroneState(int droneId, int depotId, double availableTime, int nextTripIndex) {
            this.droneId = droneId;
            this.depotId = depotId;
            this.availableTime = availableTime;
            this.nextTripIndex = nextTripIndex;
        }
    }

    private static class DroneMove {
        EVRoute launchRoute;
        EVRoute servedRoute;
        EVRoute retrieveRoute;

        int droneId;
        int tripIndex;

        int launchNodeId;
        int serveNodeId;
        int retrieveNodeId;

        int launchEVId;
        int retrieveEVId;

        double departTime;
        double arriveTime;
        double hoverTime;
        double energyUsed;

        double savingScore;
        double objectiveAfter;
    }

    private static class Stage3SearchContext {
        Map<Integer, EVRoute> routeByCustomer;
        Map<Integer, EVRoute> routeByEvId;
        Set<Integer> droneServedCustomers;
        Set<Integer> droneOperationNodes;
        Set<Integer> usedDroneIds;
        List<Integer> directCustomersInRouteOrder;

        Map<Integer, Integer> prevNodeByCustomer;
        Map<Integer, Integer> nextNodeByCustomer;

        Stage3SearchContext(Solution sol, Set<Integer> alreadyDroneServed) {
            routeByCustomer = new HashMap<>();
            routeByEvId = new HashMap<>();
            droneServedCustomers = new HashSet<>(alreadyDroneServed);
            droneOperationNodes = new HashSet<>();
            usedDroneIds = new HashSet<>();
            directCustomersInRouteOrder = new ArrayList<>();

            prevNodeByCustomer = new HashMap<>();
            nextNodeByCustomer = new HashMap<>();

            for (EVRoute route : sol.evRoutes) {
                routeByEvId.put(route.evId, route);

                for (int i = 0; i < route.customerIds.size(); i++) {
                    int cid = route.customerIds.get(i);

                    routeByCustomer.put(cid, route);
                    directCustomersInRouteOrder.add(cid);

                    int prevNodeId = (i == 0)
                            ? route.startDepotId
                            : route.customerIds.get(i - 1);

                    int nextNodeId = (i == route.customerIds.size() - 1)
                            ? route.endDepotId
                            : route.customerIds.get(i + 1);

                    prevNodeByCustomer.put(cid, prevNodeId);
                    nextNodeByCustomer.put(cid, nextNodeId);
                }
            }

            for (DroneTrip dt : sol.allDroneTrips) {
                droneServedCustomers.add(dt.serveNodeId);
                droneOperationNodes.add(dt.launchNodeId);
                droneOperationNodes.add(dt.retrieveNodeId);
                usedDroneIds.add(dt.droneId);
            }
        }
    }
    private static class OpenDepotAssignment {
        double totalEnergy;
        int[] endDepots;
    }
}