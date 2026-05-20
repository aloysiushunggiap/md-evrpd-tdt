package util;

import algorithm.Constants;
import model.Node;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;
public class DataLoader {

    public static List<Node> customers = new ArrayList<>();
    public static List<Node> depots = new ArrayList<>();

    public static double[][] dist;
    public static Map<Integer, Integer> nodeIndex = new HashMap<>();
    public static Map<Integer, Node> customerMap = new HashMap<>();
    public static Map<Integer, Node> depotMap = new HashMap<>();
    public static Map<Integer, Node> nodeMap = new HashMap<>();
    public static int problemType = 0;
    public static int numEV = 0;
    public static int numDrone = 0;
    public static int numCustomers = 0;
    public static int numDepots = 0;
    public static String instanceName = "";

    public static Node getCustomer(int id) {
        Node n = customerMap.get(id);
        if (n == null) {
            throw new IllegalArgumentException("Customer not found: " + id);
        }
        return n;
    }

    public static Node getDepot(int id) {
        Node n = depotMap.get(id);
        if (n == null) {
            throw new IllegalArgumentException("Depot not found: " + id);
        }
        return n;
    }

    public static Node getNode(int id) {
        Node n = nodeMap.get(id);
        if (n == null) {
            throw new IllegalArgumentException("Node not found: " + id);
        }
        return n;
    }

    public static boolean isDroneEligibleCustomer(int customerId) {
        return getCustomer(customerId).demand <= Constants.DRONE_CAPACITY + 1e-9;
    }

    public static void load(String path) {
        customers.clear();
        depots.clear();
        nodeIndex.clear();
        customerMap.clear();
        depotMap.clear();
        nodeMap.clear();

        instanceName = new File(path).getName();

        List<String> lines = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.isEmpty()) {
                    lines.add(line);
                }
            }
        } catch (Exception e) {
            throw new RuntimeException("Loi doc file: " + e.getMessage(), e);
        }

        if (lines.isEmpty()) {
            throw new RuntimeException("File trong");
        }

        String[] head = lines.get(0).split("\\s+");

        problemType = Integer.parseInt(head[0]);
        numEV = Integer.parseInt(head[1]);
        numCustomers = Integer.parseInt(head[2]);
        numDepots = Integer.parseInt(head[3]);

        int customerStart = 1 + numDepots;
        int customerEnd = customerStart + numCustomers;
        int depotStart = customerEnd;
        int depotEnd = depotStart + numDepots;

        if (lines.size() < depotEnd) {
            throw new RuntimeException(
                    "File khong du dong: need >= " + depotEnd + ", got " + lines.size()
            );
        }

        for (int i = customerStart; i < customerEnd; i++) {
            String[] p = lines.get(i).split("\\s+");

            int id = Integer.parseInt(p[0]);
            double x = Double.parseDouble(p[1]);
            double y = Double.parseDouble(p[2]);
            double demand = normalizeDemandForPaperBenchmark(Double.parseDouble(p[4]));

            Node customer = new Node(id, x, y, demand, false);
            customer.readyTime = Constants.T_START;
            customer.dueTime = Constants.T_END;
            customer.serviceTime = Constants.EV_SERVICE_TIME;

            customers.add(customer);
            customerMap.put(customer.id, customer);
            nodeMap.put(customer.id, customer);
        }

        for (int i = depotStart; i < depotEnd; i++) {
            String[] p = lines.get(i).split("\\s+");

            int id = Integer.parseInt(p[0]);
            double x = Double.parseDouble(p[1]);
            double y = Double.parseDouble(p[2]);

            Node depot = new Node(id, x, y, 0.0, true);
            depot.readyTime = Constants.T_START;
            depot.dueTime = Constants.T_END;
            depot.serviceTime = 0.0;

            depots.add(depot);
            depotMap.put(depot.id, depot);
            nodeMap.put(depot.id, depot);
        }

        numDrone = Math.max(numDepots * Constants.MAX_DEPOT_DRONES_PER_DEPOT, numEV * numDepots);
        List<Node> all = new ArrayList<>();
        all.addAll(customers);
        all.addAll(depots);

        for (int i = 0; i < all.size(); i++) {
            nodeIndex.put(all.get(i).id, i);
        }

        dist = new double[all.size()][all.size()];

        for (int i = 0; i < all.size(); i++) {
            for (int j = 0; j < all.size(); j++) {
                dist[i][j] = Node.distance(all.get(i), all.get(j));
            }
        }

        long droneEligible = customers.stream()
                .filter(c -> c.demand <= Constants.DRONE_CAPACITY + 1e-9)
                .count();

        System.out.println("===== DU LIEU =====");
        System.out.println("Instance    : " + instanceName);
        System.out.println("Type        : " + problemType);
        System.out.println("Customers   : " + customers.size());
        System.out.println("Depots      : " + depots.size());
        System.out.println("EVs/depot   : " + numEV);
        System.out.println("Drone pool  : " + numDrone + " (heuristic upper bound)");
        System.out.println("Drone-able  : " + droneEligible + " / " + customers.size());
        System.out.println("===================");
    }
    private static double normalizeDemandForPaperBenchmark(double rawDemand) {
        String name = instanceName == null ? "" : instanceName.toLowerCase(Locale.ROOT);

        /*
         * Paper tạo p01-p15 từ p02/p04 và có chỉnh demand.
         * Nếu dữ liệu đầu vào của anh đã chỉnh sẵn thì KHÔNG gọi hàm này.
         */
        boolean paperInstance = name.matches("p\\d+");

        if (!paperInstance) {
            return rawDemand;
        }

        if (rawDemand >= 11.0 && rawDemand <= 15.0) {
            return rawDemand - 10.0;
        }

        if (rawDemand >= 16.0 && rawDemand <= 19.0) {
            return rawDemand - 15.0;
        }

        return rawDemand;
    }
    public static double distance(int id1, int id2) {
        Integer i = nodeIndex.get(id1);
        Integer j = nodeIndex.get(id2);

        if (i == null || j == null) {
            throw new IllegalArgumentException(
                    "Distance node not found: " + id1 + ", " + id2
            );
        }

        return dist[i][j];
    }
}