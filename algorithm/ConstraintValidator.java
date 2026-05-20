package algorithm;

import util.EnergyUtil;
import util.TimeUtil;

/**
 * Lớp tiện ích tính nhanh thời gian và năng lượng.
 *
 * Lưu ý:
 * - EV phải tính theo departureTime thực tế, không dùng T_START cố định.
 * - Drone không phụ thuộc traffic.
 */
public class ConstraintValidator {

    private ConstraintValidator() {
    }

    /**
     * Tính thời gian di chuyển.
     *
     * @param distance      khoảng cách hình học (km), với EV sẽ tự nhân detour
     * @param departureTime thời điểm xuất phát thực tế (giờ)
     * @param isDrone       true nếu là drone
     * @return thời gian di chuyển (giờ)
     */
    public static double calculateTravelTime(double distance, double departureTime, boolean isDrone) {
        if (distance <= 0) return 0.0;

        if (isDrone) {
            return TimeUtil.droneTravelTime(distance);
        }

        return TimeUtil.travelTime(departureTime, distance * Constants.DETOUR_COEFF);
    }

    /**
     * Tính năng lượng tiêu thụ.
     *
     * @param distance      khoảng cách hình học (km), với EV sẽ tự nhân detour
     * @param departureTime thời điểm xuất phát thực tế (giờ)
     * @param load          tải hiện tại (kg), chỉ áp dụng với EV
     * @param isDrone       true nếu là drone
     * @return năng lượng tiêu thụ (kWh)
     */
    public static double calculateEnergy(double distance, double departureTime, double load, boolean isDrone) {
        if (distance <= 0) return 0.0;

        if (isDrone) {
            // Hàm này chỉ là quick helper cho drone 1 chặng.
            // Với drone multi-leg launch->serve->retrieve, phải dùng EnergyUtil.droneEnergy(...)
            return EnergyUtil.droneEnergy(distance, 0.0, 0.0);
        }

        double travelTime = TimeUtil.travelTime(departureTime, distance * Constants.DETOUR_COEFF);
        return EnergyUtil.evEnergy(departureTime, travelTime, load);
    }
}