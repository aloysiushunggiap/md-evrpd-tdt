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
     * - a(t) is derived from the paper's time-dependent speed polynomial.
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

        return adaptiveSimpson(departTime, departTime + travelTime, m, eta, 1e-9, 20);
    }

    private static double power(double t, double mass, double eta) {
        double vKmh = TimeUtil.speed(t);
        double rollingTerm = mass * Constants.GRAVITY * Constants.ROLLING_COEFF;
        double aeroTerm = Constants.DRAG_COEFF * Constants.FRONTAL_AREA * vKmh * vKmh / 21.15;
        double accelTerm = Constants.ROTATING_MASS_FACTOR * mass
                * TimeUtil.accelerationMetersPerSecondSquared(t);
        return vKmh * (rollingTerm + aeroTerm + accelTerm) / (3600.0 * eta);
    }

    private static double adaptiveSimpson(double a, double b, double mass, double eta,
                                          double tolerance, int depth) {
        double c = (a + b) / 2.0;
        double whole = simpson(a, b, mass, eta);
        return adaptiveSimpson(a, b, c, whole, mass, eta, tolerance, depth);
    }

    private static double adaptiveSimpson(double a, double b, double c, double whole,
                                          double mass, double eta, double tolerance, int depth) {
        double left = simpson(a, c, mass, eta);
        double right = simpson(c, b, mass, eta);
        double delta = left + right - whole;
        if (depth <= 0 || Math.abs(delta) <= 15.0 * tolerance) {
            return left + right + delta / 15.0;
        }
        double d = (a + c) / 2.0;
        double e = (c + b) / 2.0;
        return adaptiveSimpson(a, c, d, left, mass, eta, tolerance / 2.0, depth - 1)
                + adaptiveSimpson(c, b, e, right, mass, eta, tolerance / 2.0, depth - 1);
    }

    private static double simpson(double a, double b, double mass, double eta) {
        double c = (a + b) / 2.0;
        return (b - a) / 6.0 * (power(a, mass, eta) + 4.0 * power(c, mass, eta) + power(b, mass, eta));
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
