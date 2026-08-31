package tests;

import algorithm.ConstraintChecker;
import algorithm.Decoder;
import algorithm.ScheduleEvaluator;
import model.DroneTrip;
import model.EVRoute;
import model.Node;
import model.Solution;
import util.DataLoader;
import util.EnergyUtil;
import util.TimeUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Dependency-free regression checks runnable with java -ea PaperComplianceTests. */
public final class PaperComplianceTests {
    private PaperComplianceTests() {
    }

    public static void main(String[] args) {
        testEquation48And49();
        testEnergyUsesSpeedDerivative();
        testPaperDemandPreparation();
        testLifecycleObjectiveAndConstraints();
        testConstraintFailuresAreDetected();
        testCarryThenRelaunchSatisfiesEquation41();
        testEvDoesNotWaitForLateDrone();
        testStage2SeedNearestThenChromosomeOrder();
        System.out.println("PaperComplianceTests: PASS");
    }

    private static void testEquation48And49() {
        double t = 9.25;
        double expected = -1.40e-19 * Math.pow(t, 10) + 2.11e-16 * Math.pow(t, 9)
                - 1.35e-13 * Math.pow(t, 8) + 4.78e-11 * Math.pow(t, 7)
                - 1.01e-8 * Math.pow(t, 6) + 1.30e-6 * Math.pow(t, 5)
                - 9.74130e-5 * Math.pow(t, 4) + 0.004 * Math.pow(t, 3)
                - 0.07 * Math.pow(t, 2) + 0.56 * t + 52.3;
        checkClose(expected, TimeUtil.speed(t), 1e-12, "Eq.48 polynomial");
        for (double distance : new double[]{0.1, 5.0, 25.0, 100.0}) {
            double travel = TimeUtil.travelTime(7.0, distance);
            checkClose(distance, TimeUtil.integrateSpeed(7.0, travel, 0), 1e-7, "Eq.49 residual");
        }
    }

    private static void testEnergyUsesSpeedDerivative() {
        double energyAtMorning = EnergyUtil.evEnergy(7.0, 0.5, 100.0);
        double energyAtAfternoon = EnergyUtil.evEnergy(16.0, 0.5, 100.0);
        check(energyAtMorning > 0.0 && energyAtAfternoon > 0.0, "Eq.50/51 energy must be positive");
        check(Math.abs(energyAtMorning - energyAtAfternoon) > 1e-9,
                "Eq.50/51 must respond to time-dependent speed/acceleration");
    }

    /**
     * Paper section 7: "subtract 10 and 15, for the customers who have demand 11-15 and
     * 16-19."  The rewrite is keyed off the demand value alone, never off the instance
     * name, and it is what makes enough customers fit Qu = 5 kg for the drone half of the
     * model to do anything (p01 goes from 3/50 to 23/50 drone-eligible).
     */
    private static void testPaperDemandPreparation() {
        DataLoader.load("data/p01");
        // Raw demand 15 sits in [11,15] -> 5.
        checkClose(5.0, DataLoader.getCustomer(6).demand, 1e-12,
                "demand in [11,15] must lose 10");
        // Raw demand 16 sits in [16,19] -> 1.
        checkClose(1.0, DataLoader.getCustomer(3).demand, 1e-12,
                "demand in [16,19] must lose 15");
        // Raw demand 30 is outside both bands and must survive untouched.
        checkClose(30.0, DataLoader.getCustomer(2).demand, 1e-12,
                "demand outside both bands must be read verbatim");
    }

    private static void testLifecycleObjectiveAndConstraints() {
        installToyNetwork();
        Solution solution = validToySolution();
        ScheduleEvaluator.evaluate(solution);
        check(ConstraintChecker.collectViolations(solution).isEmpty(), "toy solution must be feasible");
        DroneTrip trip = solution.allDroneTrips.get(0);
        check(trip.flightArrivalTime <= trip.rendezvousTime + 1e-12, "flight and rendezvous times split");
        check(trip.dispatchDepotId == 100 && trip.returnDepotId == 100, "depot lifecycle is explicit");
        check(solution.totalCost > 0.0, "Eq.1 four-term objective must be calculated");
    }

    private static void testConstraintFailuresAreDetected() {
        installToyNetwork();
        Solution duplicate = validToySolution();
        duplicate.evRoutes.get(0).customerIds.add(1);
        assertViolation(duplicate, "Eq.8");
        assertViolation(duplicate, "Eq.14");

        installToyNetwork();
        DataLoader.getCustomer(3).demand = 6.0;
        assertViolation(validToySolution(), "Eq.35");

        installToyNetwork();
        Solution unbalanced = validToySolution();
        unbalanced.evRoutes.get(0).endDepotId = 101;
        assertViolation(unbalanced, "Eq.2");

        installToyNetwork();
        Solution continuity = validToySolution();
        continuity.allDroneTrips.add(new DroneTrip(1, 3, 100, 3, 100, -1, -1));
        assertViolation(continuity, "Eq.41");
    }

    private static void testCarryThenRelaunchSatisfiesEquation41() {
        installToyNetwork();
        Solution solution = validToySolution();
        // The first trip is retrieved at C1.  The same EV carries the drone
        // along C1 -> C2 before it launches again.  This is paper-allowed and
        // must not be rejected merely because launchNode != retrieveNode.
        solution.allDroneTrips.set(0, new DroneTrip(1, 1, 100, 3, 1, -1, 1));
        solution.allDroneTrips.add(new DroneTrip(1, 2, 2, 3, 100, 1, -1));
        boolean equation41Violation = ConstraintChecker.collectViolations(solution)
                .stream().anyMatch(v -> v.startsWith("Eq.41"));
        check(!equation41Violation, "Eq.41 must allow EV carry before relaunch");
    }

    /**
     * Stage 2 (Section 6.1.2) resolves the paper's two sentences as follows: proximity
     * picks only the seed -- "the nearest customer is selected to initiate the first EV
     * route" -- while every later customer follows the chromosome, per "after formulating
     * the entire chromosome".
     *
     * Toy network: depot 100 at (0,0); C1/C2/C3 at x = 0.10/0.20/0.30, demand 400 each.
     * EV_CAPACITY = 1000, so exactly two customers fit and the third forces a virtual
     * depot 0 split.  Demand 400 exceeds DRONE_CAPACITY, so Stage 1 assigns no drones.
     */
    private static void testStage2SeedNearestThenChromosomeOrder() {
        installStage2ToyNetwork();
        // Cluster order [3,2,1]; C1 is promoted to seed -> traversal 1,3,2.
        Solution reversed = Decoder.decodePaperICGA(Arrays.asList(3, 2, 1));

        check(reversed.evRoutes.size() == 2, "Stage 2 must split at virtual depot 0");
        check(reversed.evRoutes.get(0).customerIds.equals(Arrays.asList(1, 3)),
                "Stage 2 must seed at the nearest customer, then follow the chromosome");
        check(reversed.evRoutes.get(1).customerIds.equals(Arrays.asList(2)),
                "customer after virtual depot must start the next EV route");
        check(ConstraintChecker.collectViolations(reversed).isEmpty(),
                "Stage 2 split must preserve a feasible solution");

        installStage2ToyNetwork();
        // Cluster order [2,3,1]; C1 is promoted to seed -> traversal 1,2,3.
        Solution shuffled = Decoder.decodePaperICGA(Arrays.asList(2, 3, 1));

        check(shuffled.evRoutes.get(0).customerIds.equals(Arrays.asList(1, 2)),
                "a different permutation must produce a different traversal");

        /*
         * Regression guard for the bug this replaced: sorting the whole cluster near-to-far
         * made decodePaperICGA constant over permutations, so every ICGA operator was a
         * no-op and the cost never moved across generations.  The decoder must stay
         * permutation-sensitive.
         */
        check(!reversed.evRoutes.get(0).customerIds
                        .equals(shuffled.evRoutes.get(0).customerIds),
                "decodePaperICGA must not be a constant function of the chromosome");
    }

    /**
     * Section 3: "unlike vehicles that must wait for the return of the drone at the
     * rendezvous node, this paper stipulates that EV services take precedence over drone
     * operation", and assumption (c): the drone "has to land on the EV before the EV
     * finishes serving the customer".
     *
     * The EV therefore must not stretch its stay to wait.  While ScheduleEvaluator pushed
     * the departure out to the rendezvous time, this check was a tautology -- the
     * departure was set to at least the rendezvous, so Eq.26 could never fire.
     *
     * C3 sits 3 km off the depot axis, so the drone launched at C1 cannot get back to C2
     * before the EV has finished serving it.
     */
    private static void testEvDoesNotWaitForLateDrone() {
        installLateDroneToyNetwork();

        Solution solution = new Solution();
        EVRoute route = new EVRoute(1, 100);
        route.endDepotId = 100;
        route.customerIds.addAll(Arrays.asList(1, 2));
        solution.evRoutes.add(route);

        DroneTrip trip = new DroneTrip(1, 1, 1, 3, 2, 1, 1);
        solution.allDroneTrips.add(trip);
        route.droneTrips.add(trip);

        boolean lateRetrieval = ConstraintChecker.collectViolations(solution)
                .stream().anyMatch(v -> v.startsWith("Eq.26"));
        check(lateRetrieval, "Eq.26 must reject a drone landing after the EV leaves");
    }

    private static Solution validToySolution() {
        Solution solution = new Solution();
        EVRoute route = new EVRoute(1, 100);
        route.endDepotId = 100;
        route.customerIds.addAll(Arrays.asList(1, 2));
        solution.evRoutes.add(route);
        solution.allDroneTrips.add(new DroneTrip(1, 1, 100, 3, 100, -1, -1));
        return solution;
    }

    private static void installToyNetwork() {
        DataLoader.customers = new ArrayList<>();
        DataLoader.depots = new ArrayList<>();
        DataLoader.nodeMap.clear();
        DataLoader.customerMap.clear();
        DataLoader.depotMap.clear();
        DataLoader.nodeIndex.clear();
        addNode(new Node(1, 0.10, 0.00, 1.0, false));
        addNode(new Node(2, 0.20, 0.00, 1.0, false));
        addNode(new Node(3, 0.15, 0.10, 1.0, false));
        addNode(new Node(100, 0.00, 0.00, 0.0, true));
        addNode(new Node(101, 0.30, 0.00, 0.0, true));

        ArrayList<Node> all = new ArrayList<>();
        all.addAll(DataLoader.customers);
        all.addAll(DataLoader.depots);
        DataLoader.dist = new double[all.size()][all.size()];
        for (int i = 0; i < all.size(); i++) {
            DataLoader.nodeIndex.put(all.get(i).id, i);
            for (int j = 0; j < all.size(); j++) DataLoader.dist[i][j] = Node.distance(all.get(i), all.get(j));
        }
    }

    /** Same layout as installToyNetwork, but C3 is far enough away to strand the drone. */
    private static void installLateDroneToyNetwork() {
        DataLoader.customers = new ArrayList<>();
        DataLoader.depots = new ArrayList<>();
        DataLoader.nodeMap.clear();
        DataLoader.customerMap.clear();
        DataLoader.depotMap.clear();
        DataLoader.nodeIndex.clear();
        addNode(new Node(1, 0.10, 0.00, 1.0, false));
        addNode(new Node(2, 0.20, 0.00, 1.0, false));
        addNode(new Node(3, 0.15, 3.00, 1.0, false));
        addNode(new Node(100, 0.00, 0.00, 0.0, true));

        ArrayList<Node> all = new ArrayList<>();
        all.addAll(DataLoader.customers);
        all.addAll(DataLoader.depots);
        DataLoader.dist = new double[all.size()][all.size()];
        for (int i = 0; i < all.size(); i++) {
            DataLoader.nodeIndex.put(all.get(i).id, i);
            for (int j = 0; j < all.size(); j++) DataLoader.dist[i][j] = Node.distance(all.get(i), all.get(j));
        }
    }

    private static void installStage2ToyNetwork() {
        DataLoader.customers = new ArrayList<>();
        DataLoader.depots = new ArrayList<>();
        DataLoader.nodeMap.clear();
        DataLoader.customerMap.clear();
        DataLoader.depotMap.clear();
        DataLoader.nodeIndex.clear();
        addNode(new Node(1, 0.10, 0.00, 400.0, false));
        addNode(new Node(2, 0.20, 0.00, 400.0, false));
        addNode(new Node(3, 0.30, 0.00, 400.0, false));
        addNode(new Node(100, 0.00, 0.00, 0.0, true));

        List<Node> all = new ArrayList<>();
        all.addAll(DataLoader.customers);
        all.addAll(DataLoader.depots);
        DataLoader.dist = new double[all.size()][all.size()];
        for (int i = 0; i < all.size(); i++) {
            DataLoader.nodeIndex.put(all.get(i).id, i);
            for (int j = 0; j < all.size(); j++) {
                DataLoader.dist[i][j] = Node.distance(all.get(i), all.get(j));
            }
        }
    }

    private static void addNode(Node node) {
        DataLoader.nodeMap.put(node.id, node);
        if (node.isDepot) {
            DataLoader.depots.add(node);
            DataLoader.depotMap.put(node.id, node);
        } else {
            DataLoader.customers.add(node);
            DataLoader.customerMap.put(node.id, node);
        }
    }

    private static void assertViolation(Solution solution, String equation) {
        boolean found = ConstraintChecker.collectViolations(solution).stream().anyMatch(v -> v.startsWith(equation));
        check(found, "expected violation " + equation);
    }

    private static void checkClose(double expected, double actual, double tolerance, String message) {
        check(Math.abs(expected - actual) <= tolerance, message + ": expected=" + expected + ", actual=" + actual);
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
