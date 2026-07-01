package algorithm;

import model.Node;
import model.Solution;
import util.DataLoader;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.Set;

/**
 * Hybrid Genetic Algorithm with Variable Neighborhood Search.
 *
 * The chromosome remains a permutation of customer ids. All objective and
 * constraint handling is still delegated to Decoder and ConstraintChecker.
 */
public class GAVNS {

    private static final Random rand = new Random(42);

    private static final double CROSSOVER_RATE = 0.85;
    private static final double MUTATION_RATE = 0.30;
    private static final double CHILD_VNS_RATE = 0.20;

    private static final int TOURNAMENT_SIZE = 3;
    private static final int VNS_NEIGHBORHOODS = 4;
    private static final int VNS_ELITE_LIMIT = 6;
    private static final int NEAREST_WINDOW = 8;

    private static List<List<Integer>> nearestCustomers = new ArrayList<>();
    private static Map<Integer, Integer> customerIndex = new HashMap<>();

    private GAVNS() {
    }

    public static Solution solve() {
        rand.setSeed(42);
        prepareCustomerLookup();

        int n = DataLoader.customers.size();
        int popSize = choosePopSize(n);
        int maxGen = chooseMaxGen(n);

        List<Individual> population = initPopulation(popSize);
        evaluate(population);
        population.sort(GAVNS::compareIndividuals);

        improveEliteWithVns(population);
        population.sort(GAVNS::compareIndividuals);

        Individual globalBest = cloneIndividual(population.get(0));
        int stagnantGenerations = 0;

        for (int gen = 0; gen < maxGen; gen++) {
            List<Individual> nextPopulation = new ArrayList<>();
            int eliteCount = Math.max(1, (int) Math.round(popSize * Constants.ELITE_RATE));

            population.sort(GAVNS::compareIndividuals);
            for (int i = 0; i < eliteCount && i < population.size(); i++) {
                nextPopulation.add(cloneIndividual(population.get(i)));
            }

            while (nextPopulation.size() < popSize) {
                Individual parent1 = tournamentSelect(population);
                Individual parent2 = tournamentSelect(population);

                List<Integer> childChromosome;
                if (rand.nextDouble() < CROSSOVER_RATE) {
                    childChromosome = orderedCrossover(parent1.chromosome, parent2.chromosome);
                } else {
                    childChromosome = new ArrayList<>(parent1.chromosome);
                }

                if (rand.nextDouble() < MUTATION_RATE) {
                    childChromosome = mutate(childChromosome);
                }

                Individual child = new Individual(childChromosome);
                evaluateIndividual(child);

                if (rand.nextDouble() < CHILD_VNS_RATE) {
                    child = improveByVns(child);
                }

                nextPopulation.add(child);
            }

            population = nextPopulation;
            improveEliteWithVns(population);
            population.sort(GAVNS::compareIndividuals);

            Individual genBest = population.get(0);
            if (compareIndividuals(genBest, globalBest) < 0) {
                globalBest = cloneIndividual(genBest);
                stagnantGenerations = 0;
            } else {
                stagnantGenerations++;
            }

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

            if (stagnantGenerations >= Constants.EARLY_STOP_PATIENCE) {
                break;
            }
        }

        return globalBest.solution;
    }

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

    private static void prepareCustomerLookup() {
        customerIndex = new HashMap<>();
        nearestCustomers = new ArrayList<>();

        for (int i = 0; i < DataLoader.customers.size(); i++) {
            customerIndex.put(DataLoader.customers.get(i).id, i);
        }

        for (Node customer : DataLoader.customers) {
            List<Integer> ids = new ArrayList<>();
            for (Node other : DataLoader.customers) {
                if (other.id != customer.id) {
                    ids.add(other.id);
                }
            }

            ids.sort((a, b) -> Double.compare(
                    DataLoader.distance(customer.id, a),
                    DataLoader.distance(customer.id, b)));
            nearestCustomers.add(ids);
        }
    }

    private static List<Individual> initPopulation(int popSize) {
        List<Individual> population = new ArrayList<>();
        List<Integer> customerIds = getCustomerIds();

        population.add(new Individual(new ArrayList<>(customerIds)));

        List<Integer> nearestSeed = nearestNeighborSeed(customerIds);
        if (!nearestSeed.isEmpty()) {
            population.add(new Individual(nearestSeed));
        }

        while (population.size() < popSize) {
            List<Integer> chromosome = new ArrayList<>(customerIds);
            Collections.shuffle(chromosome, rand);
            population.add(new Individual(chromosome));
        }

        return population;
    }

    private static List<Integer> getCustomerIds() {
        List<Integer> customerIds = new ArrayList<>();
        for (Node c : DataLoader.customers) {
            customerIds.add(c.id);
        }
        return customerIds;
    }

    private static List<Integer> nearestNeighborSeed(List<Integer> customerIds) {
        if (customerIds.isEmpty()) {
            return new ArrayList<>();
        }

        Set<Integer> unused = new HashSet<>(customerIds);
        List<Integer> result = new ArrayList<>();
        int current = customerIds.get(rand.nextInt(customerIds.size()));

        while (!unused.isEmpty()) {
            result.add(current);
            unused.remove(current);

            int next = -1;
            double bestDistance = Double.MAX_VALUE;
            for (int candidate : unused) {
                double distance = DataLoader.distance(current, candidate);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    next = candidate;
                }
            }

            if (next == -1 && !unused.isEmpty()) {
                next = unused.iterator().next();
            }
            current = next;
        }

        return result;
    }

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
            ind.fitness = sol.totalCost + sol.totalPenalty + 1_000_000.0;
        }
    }

    private static Individual tournamentSelect(List<Individual> population) {
        Individual best = population.get(rand.nextInt(population.size()));

        for (int i = 1; i < TOURNAMENT_SIZE; i++) {
            Individual candidate = population.get(rand.nextInt(population.size()));
            if (compareIndividuals(candidate, best) < 0) {
                best = candidate;
            }
        }

        return best;
    }

    private static List<Integer> orderedCrossover(List<Integer> p1, List<Integer> p2) {
        int n = p1.size();
        if (n < 2) {
            return new ArrayList<>(p1);
        }

        int left = rand.nextInt(n);
        int right = rand.nextInt(n);
        if (left > right) {
            int tmp = left;
            left = right;
            right = tmp;
        }

        Integer[] child = new Integer[n];
        Set<Integer> copied = new HashSet<>();

        for (int i = left; i <= right; i++) {
            child[i] = p1.get(i);
            copied.add(p1.get(i));
        }

        int write = (right + 1) % n;
        for (int offset = 0; offset < n; offset++) {
            int read = (right + 1 + offset) % n;
            int gene = p2.get(read);
            if (!copied.contains(gene)) {
                child[write] = gene;
                write = (write + 1) % n;
            }
        }

        return new ArrayList<>(Arrays.asList(child));
    }

    private static List<Integer> mutate(List<Integer> chromosome) {
        int neighborhood = rand.nextInt(VNS_NEIGHBORHOODS);
        return randomNeighbor(chromosome, neighborhood);
    }

    private static void improveEliteWithVns(List<Individual> population) {
        population.sort(GAVNS::compareIndividuals);
        int limit = Math.min(VNS_ELITE_LIMIT, Math.max(1, population.size() / 10));

        for (int i = 0; i < limit; i++) {
            Individual improved = improveByVns(population.get(i));
            if (compareIndividuals(improved, population.get(i)) < 0) {
                population.set(i, improved);
            }
        }
    }

    private static Individual improveByVns(Individual start) {
        Individual best = cloneIndividual(start);
        int k = 0;

        while (k < VNS_NEIGHBORHOODS) {
            Individual candidate = bestNeighbor(best.chromosome, k);

            if (compareIndividuals(candidate, best) < 0) {
                best = candidate;
                k = 0;
            } else {
                k++;
            }
        }

        return best;
    }

    private static Individual bestNeighbor(List<Integer> chromosome, int neighborhood) {
        Individual best = null;
        int trials = Math.max(1, Constants.OPERATOR_TRIALS);

        for (int t = 0; t < trials; t++) {
            List<Integer> candidateChromosome = randomNeighbor(chromosome, neighborhood);
            Individual candidate = new Individual(candidateChromosome);
            evaluateIndividual(candidate);

            if (best == null || compareIndividuals(candidate, best) < 0) {
                best = candidate;
            }
        }

        return best;
    }

    private static List<Integer> randomNeighbor(List<Integer> chromosome, int neighborhood) {
        switch (neighborhood) {
            case 0:
                return swapMutation(chromosome);
            case 1:
                return insertionMutation(chromosome);
            case 2:
                return inversionMutation(chromosome);
            case 3:
                return nearestRelocateMutation(chromosome);
            default:
                return new ArrayList<>(chromosome);
        }
    }

    private static List<Integer> swapMutation(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);
        if (result.size() < 2) return result;

        int i = rand.nextInt(result.size());
        int j = rand.nextInt(result.size());
        while (j == i) {
            j = rand.nextInt(result.size());
        }

        Collections.swap(result, i, j);
        return result;
    }

    private static List<Integer> insertionMutation(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);
        if (result.size() < 2) return result;

        int from = rand.nextInt(result.size());
        int to = rand.nextInt(result.size());
        while (to == from) {
            to = rand.nextInt(result.size());
        }

        int customer = result.remove(from);
        if (to >= result.size()) {
            result.add(customer);
        } else {
            result.add(to, customer);
        }
        return result;
    }

    private static List<Integer> inversionMutation(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);
        if (result.size() < 2) return result;

        int left = rand.nextInt(result.size());
        int right = rand.nextInt(result.size());
        if (left > right) {
            int tmp = left;
            left = right;
            right = tmp;
        }

        while (left < right) {
            Collections.swap(result, left, right);
            left++;
            right--;
        }
        return result;
    }

    private static List<Integer> nearestRelocateMutation(List<Integer> chromosome) {
        List<Integer> result = new ArrayList<>(chromosome);
        if (result.size() < 2) return result;

        int sourceIndex = rand.nextInt(result.size());
        int customer = result.get(sourceIndex);
        Integer lookupIndex = customerIndex.get(customer);
        if (lookupIndex == null || lookupIndex >= nearestCustomers.size()) {
            return insertionMutation(chromosome);
        }

        List<Integer> related = nearestCustomers.get(lookupIndex);
        int window = Math.min(NEAREST_WINDOW, related.size());
        if (window == 0) {
            return insertionMutation(chromosome);
        }

        int anchor = related.get(rand.nextInt(window));
        result.remove(sourceIndex);

        int anchorIndex = result.indexOf(anchor);
        if (anchorIndex < 0) {
            return insertionMutation(chromosome);
        }

        int insertIndex = anchorIndex + (rand.nextBoolean() ? 0 : 1);
        result.add(insertIndex, customer);
        return result;
    }

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

        if (fa && !fb) return -1;
        if (!fa && fb) return 1;

        return Double.compare(a.fitness, b.fitness);
    }
}
