package model;

import java.util.Locale;

/**
 * Đại diện một drone trip:
 * launch -> serve -> retrieve
 *
 * Một drone có thể có nhiều trip (tripIndex tăng dần).
 */
public class DroneTrip {

    public int droneId;
    public int tripIndex;

    public int launchNodeId;
    public int serveNodeId;
    public int retrieveNodeId;

    public int launchEVId;    // -1 nếu từ depot
    public int retrieveEVId;  // -1 nếu về depot

    public double departTime;
    public double arriveTime;
    public double hoverTime;

    public double energyUsed;

    public DroneTrip(int droneId,
                     int tripIndex,
                     int launchNodeId,
                     int serveNodeId,
                     int retrieveNodeId,
                     int launchEVId,
                     int retrieveEVId) {

        this.droneId = droneId;
        this.tripIndex = tripIndex;

        this.launchNodeId = launchNodeId;
        this.serveNodeId = serveNodeId;
        this.retrieveNodeId = retrieveNodeId;

        this.launchEVId = launchEVId;
        this.retrieveEVId = retrieveEVId;

        this.departTime = 0.0;
        this.arriveTime = 0.0;
        this.hoverTime = 0.0;
        this.energyUsed = 0.0;
    }

    /**
     * Copy constructor
     */
    public DroneTrip(DroneTrip other) {
        this.droneId = other.droneId;
        this.tripIndex = other.tripIndex;

        this.launchNodeId = other.launchNodeId;
        this.serveNodeId = other.serveNodeId;
        this.retrieveNodeId = other.retrieveNodeId;

        this.launchEVId = other.launchEVId;
        this.retrieveEVId = other.retrieveEVId;

        this.departTime = other.departTime;
        this.arriveTime = other.arriveTime;
        this.hoverTime = other.hoverTime;
        this.energyUsed = other.energyUsed;
    }

    // ==========================================================
    // Helper logic (quan trọng cho Stage 3 và ConstraintChecker)
    // ==========================================================

    /**
     * Drone này launch từ depot?
     */
    public boolean launchedFromDepot() {
        return launchEVId < 0;
    }

    /**
     * Drone này retrieve về depot?
     */
    public boolean retrievedToDepot() {
        return retrieveEVId < 0;
    }

    /**
     * Node hiện tại của drone sau khi kết thúc trip này
     */
    public int finalNode() {
        return retrieveNodeId;
    }

    /**
     * Owner hiện tại của drone sau trip này
     * -1 = depot
     */
    public int finalOwnerEV() {
        return retrieveEVId;
    }

    /**
     * Kiểm tra continuity giữa 2 trip liên tiếp
     */
    public boolean isContinuousWith(DroneTrip next) {
        if (next == null) return false;

        // node continuity
        if (this.retrieveNodeId != next.launchNodeId) {
            return false;
        }

        // owner continuity
        if (this.retrieveEVId > 0) {
            return next.launchEVId == this.retrieveEVId;
        } else {
            // từ depot
            return next.launchEVId < 0;
        }
    }

    // ==========================================================
    // Debug / logging
    // ==========================================================

    public String compactString() {
        String type;

        if (launchEVId < 0 && retrieveEVId < 0) {
            type = "Depot-C-Depot";
        } else if (launchEVId < 0) {
            type = "Depot-C-EV" + retrieveEVId;
        } else if (retrieveEVId < 0) {
            type = "EV" + launchEVId + "-C-Depot";
        } else if (launchEVId == retrieveEVId) {
            type = "EV" + launchEVId + "-C-EV" + retrieveEVId;
        } else {
            type = "EV" + launchEVId + "-C-EV" + retrieveEVId;
        }

        return String.format(Locale.US,
                "D%d.%d[%s: %d->%d->%d]",
                droneId,
                tripIndex,
                type,
                launchNodeId,
                serveNodeId,
                retrieveNodeId
        );
    }

    public String toOperationalString() {
        String launchOwner = launchEVId < 0
                ? "Depot D" + launchNodeId
                : "EV" + launchEVId;

        String retrieveOwner = retrieveEVId < 0
                ? "Depot D" + retrieveNodeId
                : "EV" + retrieveEVId;

        String type;

        if (launchEVId < 0 && retrieveEVId < 0) {
            type = "Depot -> Customer -> Depot";
        } else if (launchEVId < 0) {
            type = "Depot -> Customer -> EV";
        } else if (retrieveEVId < 0) {
            type = "EV -> Customer -> Depot";
        } else if (launchEVId == retrieveEVId) {
            type = "EV -> Customer -> Same EV";
        } else {
            type = "EV -> Customer -> Other EV";
        }

        return String.format(Locale.US,
                "Drone%d Trip%d [%s]: launch(%s @ node %d) -> serve(C%d) -> retrieve(%s @ node %d), depart=%.3f, arrive=%.3f, hover=%.3f, energy=%.5f",
                droneId,
                tripIndex,
                type,
                launchOwner,
                launchNodeId,
                serveNodeId,
                retrieveOwner,
                retrieveNodeId,
                departTime,
                arriveTime,
                hoverTime,
                energyUsed
        );
    }

    @Override
    public String toString() {
        return toOperationalString();
    }
}