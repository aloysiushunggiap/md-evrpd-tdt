package model;

import util.DataLoader;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

/**
 * Đại diện một EV route.
 *
 * QUAN TRỌNG:
 * - droneTrips KHÔNG được copy object
 * - tất cả phải dùng cùng reference với Solution.allDroneTrips
 */
public class EVRoute {

    public int evId;
    public int startDepotId;
    public int endDepotId;
    public List<Integer> customerIds;

    // ⚠️ CHỈ LƯU REFERENCE, KHÔNG COPY
    public List<DroneTrip> droneTrips;

    public List<Double> arrivalTimes;
    public List<Double> departureTimes;
    public List<Double> loads;
    public Map<Integer, Double> departureTimeByCustomerNode;

    public double energyUsed;
    public double totalCost;
    public boolean feasible;

    public EVRoute(int evId, int startDepotId) {
        this.evId = evId;
        this.startDepotId = startDepotId;
        this.endDepotId = startDepotId;

        this.customerIds = new ArrayList<>();
        this.droneTrips = new ArrayList<>();

        this.arrivalTimes = new ArrayList<>();
        this.departureTimes = new ArrayList<>();
        this.loads = new ArrayList<>();
        this.departureTimeByCustomerNode = new HashMap<>();
    }

    /**
     * Copy constructor
     *
     * ⚠️ KHÔNG clone DroneTrip
     */
    public EVRoute(EVRoute other) {
        this.evId = other.evId;
        this.startDepotId = other.startDepotId;
        this.endDepotId = other.endDepotId;

        this.customerIds = new ArrayList<>(other.customerIds);

        // ⚠️ QUAN TRỌNG: giữ nguyên reference
        this.droneTrips = new ArrayList<>(other.droneTrips);

        this.arrivalTimes = new ArrayList<>(other.arrivalTimes);
        this.departureTimes = new ArrayList<>(other.departureTimes);
        this.loads = new ArrayList<>(other.loads);
        this.departureTimeByCustomerNode =
                new HashMap<>(other.departureTimeByCustomerNode);
        this.energyUsed = other.energyUsed;
        this.totalCost = other.totalCost;
        this.feasible = other.feasible;
    }

    /**
     * Tổng demand EV phải chịu:
     * - khách EV phục vụ
     * - khách drone launch từ EV này
     */
    public double totalDemandServedByEVAndDrone() {
        double sum = 0.0;

        for (int cid : customerIds) {
            sum += DataLoader.getCustomer(cid).demand;
        }

        for (DroneTrip dt : droneTrips) {
            if (dt.launchEVId == this.evId) {
                sum += DataLoader.getCustomer(dt.serveNodeId).demand;
            }
        }

        return sum;
    }

    /**
     * Demand chỉ tính EV trực tiếp
     */
    public double totalEVDemand() {
        double sum = 0.0;
        for (int cid : customerIds) {
            sum += DataLoader.getCustomer(cid).demand;
        }
        return sum;
    }

    /**
     * Số drone launch tại node
     */
    public int launchCountAtNode(int nodeId) {
        int count = 0;
        for (DroneTrip dt : droneTrips) {
            if (dt.launchEVId == this.evId && dt.launchNodeId == nodeId) {
                count++;
            }
        }
        return count;
    }

    /**
     * Số drone retrieve tại node
     */
    public int retrieveCountAtNode(int nodeId) {
        int count = 0;
        for (DroneTrip dt : droneTrips) {
            if (dt.retrieveEVId == this.evId && dt.retrieveNodeId == nodeId) {
                count++;
            }
        }
        return count;
    }

    /**
     * EV có đi qua node không
     */
    public boolean visitsNode(int nodeId) {
        if (startDepotId == nodeId || endDepotId == nodeId) {
            return true;
        }
        return customerIds.contains(nodeId);
    }

    /**
     * Thời điểm EV rời node
     */
    public double getDepartureAtNode(int nodeId) {
        if (arrivalTimes.isEmpty() || departureTimes.isEmpty()) {
            return 0.0;
        }

        if (nodeId == startDepotId) {
            return departureTimes.get(0);
        }

        Double t = departureTimeByCustomerNode.get(nodeId);
        if (t != null) {
            return t;
        }

        if (nodeId == endDepotId) {
            return arrivalTimes.get(arrivalTimes.size() - 1);
        }

        return 0.0;
    }

    /**
     * Route rỗng có thể xóa
     */
    public boolean isRemovableEmptyRoute() {
        return customerIds.isEmpty() && droneTrips.isEmpty();
    }
}