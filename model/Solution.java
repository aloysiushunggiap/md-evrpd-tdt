package model;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.HashMap;
/**
 * Lời giải đầy đủ cho MD-EVRPD-TDT.
 *
 * QUAN TRỌNG:
 * - allDroneTrips là danh sách trung tâm của toàn bộ drone trips
 * - EVRoute.droneTrips chỉ được trỏ tới cùng object trong allDroneTrips
 * - copy constructor phải remap reference, không được clone lệch
 */
public class Solution {

    public List<EVRoute> evRoutes;
    public List<DroneTrip> allDroneTrips;

    public double totalCost;
    public double totalPenalty;
    public boolean feasible;

    public Solution() {
        this.evRoutes = new ArrayList<>();
        this.allDroneTrips = new ArrayList<>();
        this.totalCost = Double.MAX_VALUE;
        this.totalPenalty = Double.MAX_VALUE;
        this.feasible = false;
    }

    /**
     * Copy constructor an toàn.
     *
     * Không được copy route và drone list độc lập.
     * Phải:
     * 1. clone tất cả DroneTrip trong allDroneTrips
     * 2. tạo map oldTrip -> newTrip
     * 3. copy EVRoute và remap route.droneTrips sang newTrip tương ứng
     */
    public Solution(Solution other) {
        this.evRoutes = new ArrayList<>();
        this.allDroneTrips = new ArrayList<>();

        Map<DroneTrip, DroneTrip> tripMap = new IdentityHashMap<>();

        // 1. Clone all drone trips trước
        for (DroneTrip oldTrip : other.allDroneTrips) {
            DroneTrip newTrip = new DroneTrip(oldTrip);
            this.allDroneTrips.add(newTrip);
            tripMap.put(oldTrip, newTrip);
        }

        // 2. Copy EV routes và remap droneTrips
        for (EVRoute oldRoute : other.evRoutes) {
            EVRoute newRoute = new EVRoute(oldRoute.evId, oldRoute.startDepotId);
            newRoute.endDepotId = oldRoute.endDepotId;

            newRoute.customerIds = new ArrayList<>(oldRoute.customerIds);

            newRoute.arrivalTimes = new ArrayList<>(oldRoute.arrivalTimes);
            newRoute.departureTimes = new ArrayList<>(oldRoute.departureTimes);
            newRoute.loads = new ArrayList<>(oldRoute.loads);
            newRoute.departureTimeByCustomerNode =
                    new HashMap<>(oldRoute.departureTimeByCustomerNode);
            newRoute.arrivalTimeByCustomerNode =
                    new HashMap<>(oldRoute.arrivalTimeByCustomerNode);

            newRoute.energyUsed = oldRoute.energyUsed;
            newRoute.totalCost = oldRoute.totalCost;
            newRoute.feasible = oldRoute.feasible;

            newRoute.droneTrips = new ArrayList<>();
            for (DroneTrip oldTripRef : oldRoute.droneTrips) {
                DroneTrip mapped = tripMap.get(oldTripRef);

                if (mapped != null) {
                    newRoute.droneTrips.add(mapped);
                } else {
                    /*
                     * Fallback an toàn:
                     * Nếu route đang giữ trip không nằm trong allDroneTrips,
                     * vẫn clone và đưa vào allDroneTrips để tránh mất dữ liệu.
                     * Nhưng về nguyên tắc Decoder đúng thì không nên rơi vào đây.
                     */
                    DroneTrip newTrip = new DroneTrip(oldTripRef);
                    this.allDroneTrips.add(newTrip);
                    tripMap.put(oldTripRef, newTrip);
                    newRoute.droneTrips.add(newTrip);
                }
            }

            this.evRoutes.add(newRoute);
        }

        this.totalCost = other.totalCost;
        this.totalPenalty = other.totalPenalty;
        this.feasible = other.feasible;
    }

    public int totalEVs() {
        return evRoutes.size();
    }

    public int totalDrones() {
        return (int) allDroneTrips.stream()
                .map(dt -> dt.droneId)
                .distinct()
                .count();
    }

    public int totalCustomersServedByDrone() {
        return allDroneTrips.size();
    }

    public int totalCustomersServedByEV() {
        int total = 0;
        for (EVRoute route : evRoutes) {
            total += route.customerIds.size();
        }
        return total;
    }

    public String operationalSummary() {
        StringBuilder sb = new StringBuilder();

        sb.append(String.format(Locale.US,
                "Solution(cost=%.4f, penalty=%.4f, feasible=%b, EVs=%d, usedDrones=%d, EV-served=%d, drone-served=%d)%n",
                totalCost,
                totalPenalty,
                feasible,
                totalEVs(),
                totalDrones(),
                totalCustomersServedByEV(),
                totalCustomersServedByDrone()
        ));

        List<EVRoute> routes = new ArrayList<>(evRoutes);
        routes.sort(Comparator.comparingInt(r -> r.evId));

        for (EVRoute route : routes) {
            sb.append(routeSummary(route)).append(System.lineSeparator());
        }

        List<DroneTrip> trips = new ArrayList<>(allDroneTrips);
        trips.sort(Comparator
                .comparingInt((DroneTrip d) -> d.droneId)
                .thenComparingInt(d -> d.tripIndex));

        if (!trips.isEmpty()) {
            sb.append("Drone routes:").append(System.lineSeparator());
            for (DroneTrip dt : trips) {
                sb.append("  ")
                        .append(dt.toOperationalString())
                        .append(System.lineSeparator());
            }
        }

        return sb.toString();
    }

    private String routeSummary(EVRoute route) {
        StringBuilder sb = new StringBuilder();

        sb.append("EV").append(route.evId).append(": ");
        sb.append("D").append(route.startDepotId);

        for (int cid : route.customerIds) {
            sb.append(" -> C").append(cid);
        }

        sb.append(" -> D").append(route.endDepotId);

        if (!route.droneTrips.isEmpty()) {
            sb.append(" | droneOps: ");
            List<DroneTrip> trips = new ArrayList<>(route.droneTrips);
            trips.sort(Comparator
                    .comparingInt((DroneTrip d) -> d.droneId)
                    .thenComparingInt(d -> d.tripIndex));

            for (int i = 0; i < trips.size(); i++) {
                if (i > 0) sb.append(" ; ");
                sb.append(trips.get(i).compactString());
            }
        }

        sb.append(String.format(Locale.US,
                " | energy=%.4f | cost=%.4f | feasible=%b",
                route.energyUsed,
                route.totalCost,
                route.feasible
        ));

        return sb.toString();
    }

    @Override
    public String toString() {
        return operationalSummary();
    }
}