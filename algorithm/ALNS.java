package algorithm;

import model.EVRoute;
import model.Node;
import model.Solution;
import util.DataLoader;

import java.util.*;

/**
 * Adaptive Large Neighborhood Search (ALNS) for MD-EVRPD-TDT.
 *
 * Luồng thuật toán:
 *
 * 1. KHỞI TẠO
 *    - Decode initial solution bằng Decoder.decodePaperICGA (Stage 1–3 đầy đủ + 2-opt).
 *
 * 2. VÒNG LẶP CHÍNH:
 *    a. DESTROY — chọn operator theo trọng số adaptive (roulette wheel):
 *       [0] Random removal    — loại q customer ngẫu nhiên khỏi EV routes.
 *       [1] Shaw removal      — loại q customer địa lý gần nhau nhất.
 *       [2] Worst removal     — loại q customer có chi phí cận biên cao nhất.
 *       [3] Or-opt removal    — loại 1–3 customer liên tiếp từ route dài nhất.
 *       Drone trips phục vụ các customer bị loại cũng bị xóa.
 *    b. REPAIR — chọn operator theo trọng số adaptive:
 *       [0] Greedy insertion  — chèn từng customer vào vị trí feasible rẻ nhất.
 *       [1] Regret-2          — ưu tiên customer có regret = (2nd_best−best) lớn nhất.
 *       Feasibility: routeLevelFeasible (capacity + SoC + time window).
 *       Nếu không có route nào nhận được → mở route mới.
 *    c. STAGE 3 RE-RUN + 2-OPT:
 *       Decoder.runStage3AndEvaluate: reset drone trips, 2-opt trong từng route,
 *       gán drone trips, tính cost/feasibility.
 *    d. ACCEPTANCE (Simulated Annealing):
 *       Accept nếu delta < 0; accept với P=exp(−delta/T) nếu delta ≥ 0.
 *       T giảm dần: T = max(T_MIN, T × ALPHA).
 *    e. RESTART khi kẹt: nếu STAGNATE_LIMIT iterations không cải thiện bestCost,
 *       khởi tạo lại currentSol từ bestSol với perturbation ngẫu nhiên lớn.
 *    f. UPDATE WEIGHTS mỗi SEGMENT_SIZE iterations:
 *       Score: +3 best mới | +2 tốt hơn current | +1 SA chấp nhận | +0 từ chối.
 *       w_i = (1−decay)×w_i + decay×(score_i/uses_i).
 *
 * 3. KẾT QUẢ: bestSol — solution feasible có totalCost thấp nhất.
 */
public class ALNS {

    // Consider a full Stage 3 validation every this many generations.  The real
    // rate limiter is the "proxy improved since last validation" check in solve().
    private static final int STAGE3_INTERVAL = 1;

    private static final double T_INIT_RATIO = 0.05;
    private static final double ALPHA        = 0.9997;
    private static final double T_MIN        = 1e-4;

    private static final int    SEGMENT_SIZE    = 100;
    private static final double DECAY           = 0.15;
    private static final int    STAGNATE_LIMIT  = 500;

    private static final double SCORE_NEW_BEST       = 3.0;
    private static final double SCORE_BETTER_CURRENT = 2.0;
    private static final double SCORE_ACCEPTED        = 1.0;

    private static final double Q_MIN_RATIO = 0.10;
    private static final double Q_MAX_RATIO = 0.35;

    private static final Random rand = new Random(Constants.RANDOM_SEED + 1L);
    private static int nextEvId = 1000;

    private ALNS() {}

    // ==========================================================
    // Main solve
    // ==========================================================

    /**
     * Proxy cost: EV dispatch + EV energy only (no Stage 3).
     * Used for fast SA acceptance inside the ALNS loop.
     * Drone cost is recovered via full Stage 3 at the end.
     */
    private static double evOnlyCost(Solution sol) {
        double cost = 0;
        for (EVRoute r : sol.evRoutes) {
            if (!r.customerIds.isEmpty()) {
                cost += Constants.EV_DISPATCH_COST + r.energyUsed * Constants.ELECTRICITY_PRICE;
            }
        }
        return cost;
    }

    /** Rebuild every EV route (timing + energy) without touching drone trips or ScheduleEvaluator. */
    private static void rebuildAllRoutes(Solution sol) {
        Decoder.pruneEmptyRoutes(sol);
        for (EVRoute r : sol.evRoutes) {
            Decoder.rebuildRoute(r, r.startDepotId, r.endDepotId);
        }
    }

    /** Quick feasibility: all routes must be individually feasible (no cross-route sync). */
    private static boolean allRoutesFeasible(Solution sol) {
        for (EVRoute r : sol.evRoutes) {
            if (!Decoder.routeLevelFeasible(r)) return false;
        }
        return true;
    }

    // Population size / max gen: same thresholds as ICGA for consistent comparison.
    private static int choosePopSize(int n) {
        if (n <= 80)  return Constants.POP_SIZE_SMALL;
        if (n <= 150) return Constants.POP_SIZE_MEDIUM;
        return Constants.POP_SIZE_LARGE;
    }

    private static int chooseMaxGen(int n) {
        if (n <= 80)  return Constants.MAX_GEN_SMALL;
        if (n <= 150) return Constants.MAX_GEN_MEDIUM;
        return Constants.MAX_GEN_LARGE;
    }

    public static Solution solve() {
        rand.setSeed(Constants.RANDOM_SEED + 1L);
        Decoder.resetObjectiveEvaluations();
        long startTime = System.currentTimeMillis();
        int n = DataLoader.customers.size();

        int popSize     = choosePopSize(n);
        int maxGen      = chooseMaxGen(n);
        // Each ALNS "generation" runs the same number of evaluations as one ICGA generation.
        int itersPerGen = 3 * popSize * Constants.OPERATOR_TRIALS;

        // Full initial decode (Stage 1–3) to get a good starting solution.
        Solution initSol = Decoder.decodePaperICGA(buildInitialChromosome());
        nextEvId = initSol.evRoutes.stream().mapToInt(r -> r.evId).max().orElse(0) + 1;

        // Working copy: EV routes plus the Stage 1 depot-drone trips, no Stage 3 output
        // (lazy Stage 3).  The depot-drone trips must be kept -- their customers are in no
        // EV route, so dropping them would lose those customers for the whole run.
        Solution currentSol = new Solution(initSol);
        Decoder.clearStage3DroneTrips(currentSol);

        double currEVCost = evOnlyCost(currentSol);

        // Best EV routing found so far (by EV-only proxy cost).
        Solution bestEVSol = new Solution(currentSol);
        double bestEVCost  = currEVCost;

        // bestSol: best fully-validated (Stage 3) solution found so far.
        Solution bestSol    = initSol.feasible ? initSol : null;
        double bestRealCost = bestSol != null ? bestSol.totalCost : Double.MAX_VALUE;

        double T = T_INIT_RATIO * (initSol.feasible ? initSol.totalCost : currEVCost);

        // 4 destroy + 2 repair operators
        double[] dw     = {1.0, 1.0, 1.0, 1.0};
        double[] rw     = {1.0, 1.0};
        double[] dScore = new double[4];
        double[] rScore = new double[2];
        int[]    dUses  = new int[4];
        int[]    rUses  = new int[2];

        int stagnate    = 0;
        int globalIter  = 0;

        // Only re-run Stage 3 when the proxy actually improved since the last validation.
        double lastValidatedEVCost = Double.MAX_VALUE;

        for (int gen = 0; gen < maxGen; gen++) {

            // Per-generation tracking (AvgCost, Feasible count)
            double genCostSum   = 0;
            int    genFeasCount = 0;
            int    genEvalCount = 0;

            for (int iter = 0; iter < itersPerGen; iter++, globalIter++) {

                // Cập nhật trọng số cuối mỗi segment
                if (globalIter > 0 && globalIter % SEGMENT_SIZE == 0) {
                    for (int i = 0; i < dw.length; i++) {
                        dw[i] = (1 - DECAY) * dw[i] + DECAY * (dUses[i] > 0 ? dScore[i] / dUses[i] : 1.0);
                        dScore[i] = 0; dUses[i] = 0;
                    }
                    for (int i = 0; i < rw.length; i++) {
                        rw[i] = (1 - DECAY) * rw[i] + DECAY * (rUses[i] > 0 ? rScore[i] / rUses[i] : 1.0);
                        rScore[i] = 0; rUses[i] = 0;
                    }
                }

                // Restart khi kẹt quá lâu
                if (stagnate >= STAGNATE_LIMIT && bestEVSol != null) {
                    currentSol = perturbSolution(bestEVSol, n);
                    currEVCost = evOnlyCost(currentSol);
                    T = T_INIT_RATIO * bestEVCost * 0.5;
                    stagnate = 0;
                }

                int di = selectOperator(dw);
                int ri = selectOperator(rw);
                dUses[di]++;
                rUses[ri]++;

                int qMin = Math.max(2, (int)(n * Q_MIN_RATIO));
                int qMax = Math.max(4, (int)(n * Q_MAX_RATIO));
                int q    = qMin + rand.nextInt(Math.max(1, qMax - qMin + 1));

                Solution candidate = new Solution(currentSol);

                // --- DESTROY ---
                List<Integer> unrouted;
                switch (di) {
                    case 0: unrouted = randomRemoval(candidate, q);  break;
                    case 1: unrouted = shawRemoval(candidate, q);    break;
                    case 2: unrouted = worstRemoval(candidate, q);   break;
                    default: unrouted = orOptRemoval(candidate);     break;
                }

                if (unrouted.isEmpty()) { stagnate++; continue; }

                // --- REPAIR ---
                switch (ri) {
                    case 0: greedyInsertion(candidate, unrouted);  break;
                    default: regretInsertion(candidate, unrouted); break;
                }

                // --- FAST EVALUATE (EV routes only, no Stage 3, no 2-opt) ---
                rebuildAllRoutes(candidate);
                if (!allRoutesFeasible(candidate)) { stagnate++; continue; }
                double candidateEVCost = evOnlyCost(candidate);

                genCostSum += candidateEVCost;
                genFeasCount++;
                genEvalCount++;

                // --- ACCEPTANCE (Simulated Annealing on EV-only cost proxy) ---
                double reward = 0;
                boolean accept = false;
                double delta = candidateEVCost - currEVCost;

                if (delta < -1e-9) {
                    accept = true;
                    if (candidateEVCost < bestEVCost - 1e-9) {
                        reward     = SCORE_NEW_BEST;
                        bestEVSol  = new Solution(candidate);
                        bestEVCost = candidateEVCost;
                        stagnate   = 0;
                    } else {
                        reward = SCORE_BETTER_CURRENT;
                        stagnate++;
                    }
                } else if (T > T_MIN && Math.exp(-delta / T) > rand.nextDouble()) {
                    accept = true;
                    reward = SCORE_ACCEPTED;
                    stagnate++;
                } else {
                    stagnate++;
                }

                if (accept) { currentSol = candidate; currEVCost = candidateEVCost; }

                dScore[di] += reward;
                rScore[ri] += reward;
                T = Math.max(T_MIN, T * ALPHA);
            }

            // End of generation: validate the best EV routing with full Stage 3.
            // Only worth doing when the proxy improved since the last validation —
            // re-running Stage 3 on an unchanged routing costs time and adds nothing.
            // Feasibility is decided by ConstraintChecker via probe.feasible, not by
            // second-guessing the EV count here.
            if (gen % STAGE3_INTERVAL == 0 && bestEVCost < lastValidatedEVCost - 1e-9) {
                lastValidatedEVCost = bestEVCost;
                Solution probe = new Solution(bestEVSol);
                Decoder.runStage3AndEvaluate(probe);
                if (probe.feasible && probe.totalCost < bestRealCost - 1e-9) {
                    bestSol      = probe;
                    bestRealCost = probe.totalCost;
                }
            }

            // Print Gen line in same column format as ICGA, but note the scales differ:
            //   Cost    = full Eq.1 objective (EV+drone dispatch + EV+drone energy), Stage-3 validated
            //   AvgCost = mean EV-only proxy (EV dispatch + EV energy) over this generation's candidates
            //   AvgFit  = best EV-only proxy so far — same scale as AvgCost, comparable to it
            // AvgCost/AvgFit are therefore NOT comparable to Cost.
            Solution printSol = bestSol != null ? bestSol : initSol;
            double   showCost = bestRealCost < Double.MAX_VALUE ? bestRealCost : bestEVCost;
            double   avgCost  = genEvalCount > 0 ? genCostSum / genEvalCount : bestEVCost;
            System.out.printf(Locale.US,
                "Gen %3d | Cost=%.2f | AvgCost=%.2f | AvgFit=%.2f | Feasible=%d/%d | EV=%d | Drone=%d | EV-Served=%d | D-Served=%d | t=%ds%n",
                gen, showCost, avgCost, bestEVCost,
                genFeasCount, itersPerGen,
                printSol.totalEVs(), printSol.totalDrones(),
                printSol.totalCustomersServedByEV(), printSol.totalCustomersServedByDrone(),
                (System.currentTimeMillis() - startTime) / 1000
            );
        }

        // Final Stage 3 + 2-opt on best EV routing found.
        System.out.println("Running final Stage 3 + 2-opt on best EV routing...");
        Solution finalProbe = new Solution(bestEVSol);
        Decoder.applyLocalSearchAndEvaluate(finalProbe);
        if (finalProbe.feasible && finalProbe.totalCost < bestRealCost - 1e-9) {
            bestSol = finalProbe;
        }

        return bestSol != null ? bestSol : initSol;
    }

    // ==========================================================
    // Destroy operators
    // ==========================================================

    private static List<Integer> randomRemoval(Solution sol, int q) {
        List<Integer> all = evServedCustomers(sol);
        Collections.shuffle(all, rand);
        return removeCustomers(sol, all.subList(0, Math.min(q, all.size())));
    }

    private static List<Integer> shawRemoval(Solution sol, int q) {
        List<Integer> all = evServedCustomers(sol);
        if (all.isEmpty()) return Collections.emptyList();
        Set<Integer> toRemove = new LinkedHashSet<>();
        toRemove.add(all.get(rand.nextInt(all.size())));
        while (toRemove.size() < q && toRemove.size() < all.size()) {
            int ref = new ArrayList<>(toRemove).get(rand.nextInt(toRemove.size()));
            int bestCid = -1; double bestDist = Double.MAX_VALUE;
            for (int cid : all) {
                if (toRemove.contains(cid)) continue;
                double d = DataLoader.distance(ref, cid);
                if (d < bestDist) { bestDist = d; bestCid = cid; }
            }
            if (bestCid >= 0) toRemove.add(bestCid);
        }
        return removeCustomers(sol, new ArrayList<>(toRemove));
    }

    private static List<Integer> worstRemoval(Solution sol, int q) {
        List<Integer> toRemove = new ArrayList<>();
        for (int k = 0; k < q; k++) {
            int worstCid = -1; double worstCost = -1;
            for (EVRoute route : sol.evRoutes) {
                List<Integer> ids = route.customerIds;
                for (int i = 0; i < ids.size(); i++) {
                    int cid  = ids.get(i);
                    int prev = (i == 0)              ? route.startDepotId : ids.get(i - 1);
                    int next = (i == ids.size() - 1) ? route.endDepotId   : ids.get(i + 1);
                    double m = DataLoader.distance(prev, cid) + DataLoader.distance(cid, next)
                             - DataLoader.distance(prev, next);
                    if (m > worstCost) { worstCost = m; worstCid = cid; }
                }
            }
            if (worstCid < 0) break;
            toRemove.add(worstCid);
            for (EVRoute r : sol.evRoutes) r.customerIds.remove((Integer) worstCid);
        }
        return removeCustomers(sol, toRemove);
    }

    /**
     * Or-opt removal: lấy 1–3 customer liên tiếp từ route có nhiều customer nhất
     * (route dài nhất theo số lượng customer) để tạo cơ hội merge.
     */
    private static List<Integer> orOptRemoval(Solution sol) {
        // Chọn route có nhiều customer nhất làm nguồn
        EVRoute source = null;
        for (EVRoute r : sol.evRoutes) {
            if (source == null || r.customerIds.size() > source.customerIds.size()) source = r;
        }
        if (source == null || source.customerIds.size() < 2) return Collections.emptyList();

        int segLen = Math.min(1 + rand.nextInt(3), source.customerIds.size());
        int startIdx = rand.nextInt(source.customerIds.size() - segLen + 1);
        List<Integer> segment = new ArrayList<>(source.customerIds.subList(startIdx, startIdx + segLen));
        return removeCustomers(sol, segment);
    }

    // ==========================================================
    // Repair operators
    // ==========================================================

    private static void greedyInsertion(Solution sol, List<Integer> unrouted) {
        List<Integer> remaining = new ArrayList<>(unrouted);
        Collections.shuffle(remaining, rand);
        for (int cid : remaining) insertBest(sol, cid);
    }

    private static void regretInsertion(Solution sol, List<Integer> unrouted) {
        List<Integer> remaining = new ArrayList<>(unrouted);
        while (!remaining.isEmpty()) {
            int bestIdx = -1;
            double bestRegret = -Double.MAX_VALUE;
            InsertionPos bestPos = null;
            for (int i = 0; i < remaining.size(); i++) {
                int cid = remaining.get(i);
                InsertionPos p1 = bestInsertionPos(sol, cid);
                InsertionPos p2 = secondBestInsertionPos(sol, cid, p1);
                double c1 = p1 != null ? p1.delta : Double.MAX_VALUE;
                double c2 = p2 != null ? p2.delta : c1;
                double regret = c2 - c1;
                if (regret > bestRegret) { bestRegret = regret; bestIdx = i; bestPos = p1; }
            }
            if (bestIdx < 0) break;
            int bestCid = remaining.remove(bestIdx);
            if (bestPos != null) applyInsertion(sol, bestPos);
            else openNewRoute(sol, bestCid);
        }
    }

    // ==========================================================
    // Insertion helpers
    // ==========================================================

    private static void insertBest(Solution sol, int cid) {
        InsertionPos best = bestInsertionPos(sol, cid);
        if (best != null) applyInsertion(sol, best);
        else openNewRoute(sol, cid);
    }

    private static InsertionPos bestInsertionPos(Solution sol, int cid) {
        InsertionPos best = null;
        for (EVRoute route : sol.evRoutes) {
            InsertionPos p = bestPosInRoute(route, cid);
            if (p != null && (best == null || p.delta < best.delta)) best = p;
        }
        return best;
    }

    private static InsertionPos secondBestInsertionPos(Solution sol, int cid, InsertionPos excluded) {
        InsertionPos best = null;
        for (EVRoute route : sol.evRoutes) {
            InsertionPos p = bestPosInRoute(route, cid);
            if (p == null) continue;
            if (excluded != null && p.route == excluded.route && p.pos == excluded.pos) continue;
            if (best == null || p.delta < best.delta) best = p;
        }
        return best;
    }

    // Number of top Euclidean-delta positions to verify with rebuildRoute
    private static final int INSERT_VERIFY_TOP = 2;

    private static InsertionPos bestPosInRoute(EVRoute route, int cid) {
        List<Integer> ids = route.customerIds;
        int size = ids.size();
        Node cidNode = DataLoader.getCustomer(cid);

        // Step 1: rank ALL positions by cheap Euclidean insertion delta.
        List<int[]> ranked = new ArrayList<>(size + 1);  // [pos, euclidDelta*1e6]
        for (int pos = 0; pos <= size; pos++) {
            int prev = (pos == 0)    ? route.startDepotId : ids.get(pos - 1);
            int next = (pos == size) ? route.endDepotId   : ids.get(pos);
            double delta = DataLoader.distance(prev, cid) + DataLoader.distance(cid, next)
                         - DataLoader.distance(prev, next);
            ranked.add(new int[]{pos, (int)(delta * 1e3)});
        }
        ranked.sort(Comparator.comparingInt(a -> a[1]));

        // Step 2: rebuild only the top-INSERT_VERIFY_TOP positions to check real feasibility.
        InsertionPos best = null;
        for (int k = 0; k < Math.min(INSERT_VERIFY_TOP, ranked.size()); k++) {
            int pos = ranked.get(k)[0];
            EVRoute test = new EVRoute(route);
            test.customerIds.add(pos, cid);
            Decoder.rebuildRoute(test, test.startDepotId, test.endDepotId);
            if (!Decoder.routeLevelFeasible(test)) continue;
            double delta = test.energyUsed - route.energyUsed;
            if (best == null || delta < best.delta) best = new InsertionPos(route, cid, pos, delta);
        }
        return best;
    }

    private static void applyInsertion(Solution sol, InsertionPos p) {
        p.route.customerIds.add(p.pos, p.customerId);
        Decoder.rebuildRoute(p.route, p.route.startDepotId, p.route.endDepotId);
    }

    private static void openNewRoute(Solution sol, int cid) {
        int depotId = nearestDepotId(cid);
        EVRoute newRoute = new EVRoute(nextEvId++, depotId);
        newRoute.customerIds.add(cid);
        newRoute.endDepotId = depotId;
        Decoder.rebuildRoute(newRoute, depotId, depotId);
        sol.evRoutes.add(newRoute);
    }

    // ==========================================================
    // Remove helpers
    // ==========================================================

    private static List<Integer> evServedCustomers(Solution sol) {
        List<Integer> result = new ArrayList<>();
        for (EVRoute route : sol.evRoutes) result.addAll(route.customerIds);
        return result;
    }

    private static List<Integer> removeCustomers(Solution sol, List<Integer> toRemove) {
        List<Integer> list = new ArrayList<>(toRemove);
        Set<Integer> removeSet = new HashSet<>(list);
        for (EVRoute route : sol.evRoutes) route.customerIds.removeIf(removeSet::contains);
        sol.allDroneTrips.removeIf(dt -> removeSet.contains(dt.serveNodeId));
        for (EVRoute route : sol.evRoutes) route.droneTrips.removeIf(dt -> removeSet.contains(dt.serveNodeId));
        return list;
    }

    // ==========================================================
    // Perturbation restart
    // ==========================================================

    /**
     * Double-bridge perturbation trên bestSol để thoát khỏi local optimum.
     * Chọn 2 route ngẫu nhiên, hoán đổi một đoạn giữa chúng, sau đó rebuild.
     */
    private static Solution perturbSolution(Solution best, int n) {
        Solution sol = new Solution(best);
        List<EVRoute> routes = sol.evRoutes;
        if (routes.size() < 2) {
            // Chỉ có 1 route: random removal lớn
            List<Integer> all = evServedCustomers(sol);
            int q = Math.max(3, n / 5);
            Collections.shuffle(all, rand);
            List<Integer> removed = removeCustomers(sol, all.subList(0, Math.min(q, all.size())));
            greedyInsertion(sol, removed);
        } else {
            // Swap đoạn random giữa 2 route
            int ia = rand.nextInt(routes.size());
            int ib = rand.nextInt(routes.size());
            while (ib == ia) ib = rand.nextInt(routes.size());
            EVRoute ra = routes.get(ia);
            EVRoute rb = routes.get(ib);
            if (!ra.customerIds.isEmpty() && !rb.customerIds.isEmpty()) {
                int lenA = 1 + rand.nextInt(Math.max(1, ra.customerIds.size() / 2));
                int startA = rand.nextInt(ra.customerIds.size() - lenA + 1);
                List<Integer> segA = new ArrayList<>(ra.customerIds.subList(startA, startA + lenA));
                ra.customerIds.subList(startA, startA + lenA).clear();
                greedyInsertion(sol, segA);
            }
        }
        return sol;
    }

    // ==========================================================
    // Helpers
    // ==========================================================

    private static List<Integer> buildInitialChromosome() {
        List<Node> customers = new ArrayList<>(DataLoader.customers);
        customers.sort((a, b) -> {
            Node da = nearestDepot(a), db = nearestDepot(b);
            if (da.id != db.id) return Integer.compare(da.id, db.id);
            return Double.compare(DataLoader.distance(a.id, da.id), DataLoader.distance(b.id, db.id));
        });
        List<Integer> chr = new ArrayList<>();
        for (Node c : customers) chr.add(c.id);
        return chr;
    }

    private static Node nearestDepot(Node c) {
        Node best = null; double bestDist = Double.MAX_VALUE;
        for (Node d : DataLoader.depots) {
            double dist = DataLoader.distance(c.id, d.id);
            if (dist < bestDist) { bestDist = dist; best = d; }
        }
        return best;
    }

    private static int nearestDepotId(int cid) {
        Node d = nearestDepot(DataLoader.getCustomer(cid));
        return d != null ? d.id : DataLoader.depots.get(0).id;
    }

    private static int selectOperator(double[] weights) {
        double total = 0;
        for (double w : weights) total += Math.max(w, 1e-6);
        double r = rand.nextDouble() * total;
        double cum = 0;
        for (int i = 0; i < weights.length; i++) {
            cum += Math.max(weights[i], 1e-6);
            if (r <= cum) return i;
        }
        return weights.length - 1;
    }

    // ==========================================================
    // Inner class
    // ==========================================================

    private static class InsertionPos {
        final EVRoute route;
        final int customerId;
        final int pos;
        final double delta;
        InsertionPos(EVRoute route, int cid, int pos, double delta) {
            this.route = route; this.customerId = cid; this.pos = pos; this.delta = delta;
        }
    }
}
