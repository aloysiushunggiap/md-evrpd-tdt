package algorithm;

import model.Node;
import model.Solution;
import util.DataLoader;
import java.util.*;

/**
 * Improved Chaotic Genetic Algorithm theo paper:
 *
 * - chaotic initialization
 * - exchange operator
 * - insertion operator
 * - 2-opt operator
 * - better individual preservation
 *
 * Không dùng crossover/mutation truyền thống.
 */
public class ICGA {

    private static final Random rand = new Random(42);

    private ICGA() {
    }

    public static Solution solve() {
        int n = DataLoader.customers.size();

        int popSize = choosePopSize(n);
        int maxGen = chooseMaxGen(n);

        List<Individual> population = initPopulation(popSize);
        evaluate(population);

        population.sort(ICGA::compareIndividuals);

        Individual globalBest = cloneIndividual(population.get(0));
        int stagnantGenerations = 0;

        for (int gen = 0; gen < maxGen; gen++) {

            // Paper: apply operators Rk, k = 1..kmax
            for (int operator = 0; operator < 3; operator++) {
                for (int i = 0; i < population.size(); i++) {
                    Individual current = population.get(i);

                    List<Integer> newChromosome = applyOperator(current.chromosome, operator);
                    Individual candidate = new Individual(newChromosome);
                    evaluateIndividual(candidate);

                    boolean currentFeasible = current.solution != null && current.solution.feasible;
                    boolean candidateFeasible = candidate.solution != null && candidate.solution.feasible;

                    boolean accept;

                    if (candidateFeasible && !currentFeasible) {
                        accept = true;
                    } else if (!candidateFeasible && currentFeasible) {
                        accept = false;
                    } else {
                        accept = candidate.fitness + 1e-9 < current.fitness;
                    }

                    if (accept) {
                        population.set(i, candidate);
                        current = candidate;
                    }

                    if (current.fitness + 1e-9 < globalBest.fitness) {
                        globalBest = cloneIndividual(current);
                        stagnantGenerations = 0;
                    }
                }
            }

            population.sort(ICGA::compareIndividuals);

            Individual genBest = population.get(0);
            Solution bestSol = genBest.solution;

            double avgCost = population.stream()
                    .mapToDouble(ind -> ind.solution.totalCost)
                    .average()
                    .orElse(bestSol.totalCost);

            double avgFitness = population.stream()
                    .mapToDouble(ind -> ind.fitness)
                    .average()
                    .orElse(genBest.fitness);

            long feasibleCount = population.stream()
                    .filter(ind -> ind.solution.feasible)
                    .count();

            System.out.printf(Locale.US,
                    "Gen %3d | Cost=%.2f | AvgCost=%.2f | AvgFit=%.2f | Feasible=%d/%d | EV=%d | Drone=%d | EV-Served=%d | D-Served=%d%n",
                    gen,
                    bestSol.totalCost,
                    avgCost,
                    avgFitness,
                    feasibleCount,
                    population.size(),
                    bestSol.totalEVs(),
                    bestSol.totalDrones(),
                    bestSol.totalCustomersServedByEV(),
                    bestSol.totalCustomersServedByDrone()
            );

            stagnantGenerations++;
            if (stagnantGenerations >= Constants.EARLY_STOP_PATIENCE) {
                break;
            }
        }

        return globalBest.solution;
    }

    // ==========================================================
    // Population size / max generation
    // ==========================================================

    private static int choosePopSize(int n) {
        if (n <= 80) return Constants.POP_SIZE_SMALL;
        if (n <= 150) return Constants.POP_SIZE_MEDIUM;
        return Constants.POP_SIZE_LARGE;
    }

    private static int chooseMaxGen(int n) {
        if (n <= 80) return Constants.MAX_GEN_SMALL;
        if (n <= 150) return Constants.MAX_GEN_MEDIUM;
        return Constants.MAX_GEN_LARGE;
    }

    // ==========================================================
    // Chaotic initialization
    // ==========================================================

    private static List<Individual> initPopulation(int popSize) {
        List<Individual> population = new ArrayList<>();

        int n = DataLoader.customers.size();

        /*
         * Danh sách customer id theo đúng thứ tự trong DataLoader.
         * Chromosome sẽ là permutation của danh sách này.
         */
        List<Integer> customerIds = new ArrayList<>();
        for (Node c : DataLoader.customers) {
            customerIds.add(c.id);
        }

        /*
         * Tối ưu runtime:
         * Trước đây comparator trong sort gọi customerIds.indexOf(a),
         * mỗi lần indexOf là O(n), làm sort chromosome bị chậm.
         *
         * Bây giờ map customerId -> index được tạo một lần,
         * comparator lấy index O(1).
         */
        Map<Integer, Integer> customerIndex = new HashMap<>();
        for (int i = 0; i < customerIds.size(); i++) {
            customerIndex.put(customerIds.get(i), i);
        }

        for (int p = 0; p < popSize; p++) {
            double x = rand.nextDouble();

            /*
             * Paper lưu ý logistic chaotic mapping mất tính ergodic
             * tại các fixed point 0.25, 0.5, 0.75.
             */
            while (isFixedPoint(x)) {
                x = rand.nextDouble();
            }

            /*
             * Sinh chuỗi chaotic theo:
             * x(n+1) = r * x(n) * (1 - x(n))
             */
            List<Double> chaoticValues = new ArrayList<>();
            for (int i = 0; i < n; i++) {
                x = Constants.CHAOTIC_R * x * (1.0 - x);
                chaoticValues.add(x);
            }

            /*
             * Tạo chromosome bằng cách sort customer theo giá trị chaotic.
             * Logic không đổi so với bản cũ, chỉ thay cách lấy index nhanh hơn.
             */
            List<Integer> chromosome = new ArrayList<>(customerIds);

            chromosome.sort((a, b) -> {
                int ia = customerIndex.get(a);
                int ib = customerIndex.get(b);

                return Double.compare(
                        chaoticValues.get(ia),
                        chaoticValues.get(ib)
                );
            });

            population.add(new Individual(chromosome));
        }

        return population;
    }
    private static boolean isFixedPoint(double x) {
        return Math.abs(x - 0.25) < 1e-12
                || Math.abs(x - 0.50) < 1e-12
                || Math.abs(x - 0.75) < 1e-12;
    }

    // ==========================================================
    // Evaluation
    // ==========================================================

    private static void evaluate(List<Individual> population) {
        for (Individual ind : population) {
            evaluateIndividual(ind);
        }
    }

    private static void evaluateIndividual(Individual ind) {
        Solution sol = Decoder.decode(ind.chromosome);
        ind.solution = sol;

        if (sol.feasible) {
            ind.fitness = sol.totalCost;
        } else {
            // Paper: infeasible sau perturbation phải repair hoặc discard.
            // Vì mình chưa có repair đầy đủ, cộng phạt lớn để không cho infeasible thắng feasible.
            ind.fitness = sol.totalCost + sol.totalPenalty + 1_000_000.0;
        }
    }

    // ==========================================================
    // Operators
    // ==========================================================

    private static List<Integer> applyOperator(List<Integer> chromosome, int operator) {
        switch (operator) {
            case 0:
                return exchange(chromosome);
            case 1:
                return insertion(chromosome);
            case 2:
                return twoOpt(chromosome);
            default:
                return new ArrayList<>(chromosome);
        }
    }

    /**
     * Exchange operator:
     * chọn 2 customer và đổi vị trí.
     */
    private static List<Integer> exchange(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);

        if (result.size() < 2) {
            return result;
        }

        int i = rand.nextInt(result.size());
        int j = rand.nextInt(result.size());

        while (j == i) {
            j = rand.nextInt(result.size());
        }

        Collections.swap(result, i, j);
        return result;
    }

    /**
     * Insertion operator:
     * lấy customer i và chèn sau vị trí j.
     */
    private static List<Integer> insertion(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);

        if (result.size() < 2) {
            return result;
        }

        int i = rand.nextInt(result.size());
        int j = rand.nextInt(result.size());

        while (j == i) {
            j = rand.nextInt(result.size());
        }

        int customer = result.remove(i);

        if (j >= result.size()) {
            result.add(customer);
        } else {
            result.add(j, customer);
        }

        return result;
    }

    /**
     * 2-opt operator:
     * chọn đoạn i..j và đảo ngược.
     */
    private static List<Integer> twoOpt(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);

        if (result.size() < 2) {
            return result;
        }

        int i = rand.nextInt(result.size());
        int j = rand.nextInt(result.size());

        if (i > j) {
            int tmp = i;
            i = j;
            j = tmp;
        }

        while (i < j) {
            Collections.swap(result, i, j);
            i++;
            j--;
        }

        return result;
    }

    // ==========================================================
    // Clone helper
    // ==========================================================

    private static Individual cloneIndividual(Individual ind) {
        Individual copy = new Individual(new ArrayList<>(ind.chromosome));
        copy.fitness = ind.fitness;

        if (ind.solution != null) {
            copy.solution = new Solution(ind.solution);
        }

        return copy;
    }

    private static int compareIndividuals(Individual a, Individual b) {
        boolean fa = a.solution != null && a.solution.feasible;
        boolean fb = b.solution != null && b.solution.feasible;

        // Paper: infeasible phải repair/discard, nên feasible luôn ưu tiên hơn infeasible
        if (fa && !fb) return -1;
        if (!fa && fb) return 1;

        return Double.compare(a.fitness, b.fitness);
    }
}
