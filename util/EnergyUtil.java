package util;

import algorithm.Constants;

/**
 * Tính năng lượng EV và drone theo paper MD-EVRPD-TDT.
 *
 * EV:
 *   Eq.(50): instantaneous power
 *   Eq.(51): energy = integral(power dt)
 *
 * Drone:
 *   E = flyTime * e1 + hoverTime * e2 + 4 * e3
 */
public class EnergyUtil {

    private EnergyUtil() {
    }

    /**
     * Tính năng lượng EV tiêu thụ trên một chặng.
     *
     * Bám theo Eq.(50)-(51) trong paper:
     * P = 1 / (3600 * etaT * etaV * etaM)
     *     * v(t)
     *     * (m*g*f + C_D*A*v(t)^2 / 21.15 + delta*m*a)
     *
     * Trong đó:
     * - m = m0 + load
     * - v(t) tính theo km/h
     * - a trong paper có xuất hiện, nhưng ở heuristic route-level hiện tại
     *   không có profile gia tốc theo từng chặng, nên dùng hằng số a = 0
     *   để tránh tự ý đưa thêm mô hình ngoài paper.
     *
     * @param departTime thời điểm xuất phát (giờ)
     * @param travelTime thời gian di chuyển trên chặng (giờ)
     * @param load tải hàng hiện tại EV đang chở (kg)
     * @return năng lượng tiêu thụ (kWh)
     */
    public static double evEnergy(double departTime, double travelTime, double load) {
        if (travelTime <= 0.0) {
            return 0.0;
        }

        double m = Constants.EV_MASS_EMPTY + Math.max(0.0, load);
        double eta = Constants.ETA_T * Constants.ETA_V * Constants.ETA_M;

        int steps = 12;
        if (steps % 2 == 1) {
            steps++;
        }

        double h = travelTime / steps;
        double weightedPowerSum = 0.0;

        for (int i = 0; i <= steps; i++) {
            double t = departTime + i * h;

            // Paper dùng v(t) theo km/h
            double vKmh = TimeUtil.speed(t);

            // Eq.(50):
            // m*g*f
            double rollingTerm = m * Constants.GRAVITY * Constants.ROLLING_COEFF;

            // C_D * A * v^2 / 21.15
            double aeroTerm = Constants.DRAG_COEFF
                    * Constants.FRONTAL_AREA
                    * vKmh * vKmh / 21.15;

            // delta * m * a
            double accelTerm = Constants.ROTATING_MASS_FACTOR
                    * m
                    * Constants.EV_ACCELERATION;

            // instantaneous power theo Eq.(50), đơn vị kW khi dùng hệ số 3600 như paper
            double powerKw = (1.0 / (3600.0 * eta))
                    * vKmh
                    * (rollingTerm + aeroTerm + accelTerm);

            double weight;
            if (i == 0 || i == steps) {
                weight = 1.0;
            } else if (i % 2 == 0) {
                weight = 2.0;
            } else {
                weight = 4.0;
            }

            weightedPowerSum += weight * powerKw;
        }

        // Simpson integration: kW * h = kWh
        return weightedPowerSum * h / 3.0;
    }

    /**
     * Năng lượng drone cho một chuyến launch -> serve -> retrieve.
     *
     * Theo objective (1) của paper:
     *   ((l_ih + l_hj) / v_u) * e1 + hoverTime * e2 + 4 * e3
     *
     * @param distLaunchToServe khoảng cách launch -> serve (km)
     * @param distServeToRetrieve khoảng cách serve -> retrieve (km)
     * @param hoverTime thời gian hover (giờ)
     * @return năng lượng drone (kWh)
     */
    public static double droneEnergy(double distLaunchToServe,
                                     double distServeToRetrieve,
                                     double hoverTime) {

        double flyTime = (distLaunchToServe + distServeToRetrieve) / Constants.DRONE_SPEED;

        return flyTime * Constants.DRONE_E1
                + Math.max(0.0, hoverTime) * Constants.DRONE_E2
                + 4.0 * Constants.DRONE_E3;
    }

    /**
     * Kiểm tra nhanh drone có đủ pin cho chuyến launch -> serve -> retrieve không,
     * chưa tính hover.
     *
     * Dùng cho screening candidate trước khi evaluate chi tiết.
     */
    public static boolean droneHasEnoughEnergy(double distLaunchToServe,
                                               double distServeToRetrieve) {
        double energy = droneEnergy(distLaunchToServe, distServeToRetrieve, 0.0);
        double remain = Constants.DRONE_BATTERY - energy;
        return remain >= Constants.DRONE_MIN_ENERGY - 1e-9;
    }
}