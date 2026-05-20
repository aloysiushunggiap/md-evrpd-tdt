package util;

import algorithm.Constants;

/**
 * Tính thời gian di chuyển EV theo paper:
 * - Eq.(48): tốc độ phụ thuộc thời gian
 * - Eq.(49): quãng đường = tích phân vận tốc theo thời gian
 *
 * Bản tối ưu:
 * - speed(t) dùng Horner thay vì pow lặp.
 * - distanceIntegral dùng nguyên hàm của đa thức tốc độ thay vì Simpson.
 *
 * Không đổi mô hình paper.
 */
public class TimeUtil {

    private TimeUtil() {
    }

    /**
     * Eq.(48): tốc độ EV theo thời gian trong ngày.
     * t tính theo giờ.
     */
    public static double speed(double t) {
        /*
         * Horner form của:
         * -1.40e-19 t^10 + 2.11e-16 t^9 - 1.35e-13 t^8
         * + 4.78e-11 t^7 - 1.01e-8 t^6 + 1.30e-6 t^5
         * - 9.74130e-5 t^4 + 0.004 t^3 - 0.07 t^2
         * + 0.56 t + 52.3
         */
        double v = ((((((((((-1.40e-19 * t
                + 2.11e-16) * t
                - 1.35e-13) * t
                + 4.78e-11) * t
                - 1.01e-8) * t
                + 1.30e-6) * t
                - 9.74130e-5) * t
                + 0.004) * t
                - 0.07) * t
                + 0.56) * t
                + 52.3);

        return Math.max(v, 5.0);
    }

    /**
     * Nguyên hàm của Eq.(48).
     *
     * F(t) = ∫ v(t) dt
     *
     * Trong time window [7,17], speed luôn dương và lớn hơn ngưỡng 5,
     * nên việc dùng nguyên hàm này tương ứng trực tiếp với Eq.(49).
     */
    private static double speedPrimitive(double t) {
        double a11 = -1.40e-19 / 11.0;
        double a10 =  2.11e-16 / 10.0;
        double a9  = -1.35e-13 / 9.0;
        double a8  =  4.78e-11 / 8.0;
        double a7  = -1.01e-8  / 7.0;
        double a6  =  1.30e-6  / 6.0;
        double a5  = -9.74130e-5 / 5.0;
        double a4  =  0.004 / 4.0;
        double a3  = -0.07 / 3.0;
        double a2  =  0.56 / 2.0;
        double a1  =  52.3;

        return (((((((((((a11 * t
                + a10) * t
                + a9) * t
                + a8) * t
                + a7) * t
                + a6) * t
                + a5) * t
                + a4) * t
                + a3) * t
                + a2) * t
                + a1) * t);
    }

    /**
     * Tính quãng đường EV đi được từ t0 đến t0 + dt.
     */
    public static double integrateSpeed(double t0, double dt, int steps) {
        if (dt <= 0.0) {
            return 0.0;
        }

        return speedPrimitive(t0 + dt) - speedPrimitive(t0);
    }

    /**
     * Tìm travel time T sao cho:
     * integrateSpeed(departTime, T) = distance.
     *
     * Dùng Newton, nhưng mỗi vòng giờ chỉ còn:
     * - 1 lần tính nguyên hàm
     * - 1 lần tính speed
     *
     * Không còn Simpson 12 bước trong mỗi vòng.
     */
    public static double travelTime(double departTime, double distance) {
        if (distance <= 0.0) {
            return 0.0;
        }

        double v0 = speed(departTime);
        double T = Math.max(distance / v0, distance / 120.0);

        for (int iter = 0; iter < 12; iter++) {
            double traveled = integrateSpeed(departTime, T, 0);
            double error = traveled - distance;

            if (Math.abs(error) < 1e-7) {
                break;
            }

            double vEnd = speed(departTime + T);
            T -= error / Math.max(vEnd, 1.0);

            if (T < distance / 200.0) {
                T = distance / 200.0;
            }
        }

        return Math.max(T, distance / 200.0);
    }

    /**
     * Drone có tốc độ hằng số theo paper.
     */
    public static double droneTravelTime(double distance) {
        return distance / Constants.DRONE_SPEED;
    }
}