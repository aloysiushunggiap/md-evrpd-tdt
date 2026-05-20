package model;

import algorithm.Constants;

/**
 * Node đại diện cho depot hoặc customer trong mạng giao hàng.
 *
 * Lưu ý theo paper:
 * - serviceTime tại customer là tham số mô hình ts = 3 phút
 * - không lấy serviceTime từ file benchmark
 * - depot có serviceTime = 0
 */
public class Node {
    public int id;
    public double x;
    public double y;
    public double demand;
    public boolean isDepot;

    // Hiện tại giữ lại để đồng bộ mô hình thời gian,
    // nhưng với bộ thí nghiệm paper thì chủ yếu depot dùng [T_START, T_END].
    public double readyTime;
    public double dueTime;

    // Customer dùng ts của paper, depot = 0
    public double serviceTime;

    public Node(int id, double x, double y, double demand, boolean isDepot) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.demand = demand;
        this.isDepot = isDepot;

        this.readyTime = Constants.T_START;
        this.dueTime = Constants.T_END;
        this.serviceTime = isDepot ? 0.0 : Constants.EV_SERVICE_TIME;
    }

    /**
     * Khoảng cách Euclid giữa 2 node.
     *
     * Lưu ý:
     * - Đây là l_ij gốc theo paper.
     * - EV sẽ nhân thêm hệ số detour w ở tầng khác.
     * - Drone dùng trực tiếp khoảng cách thẳng này.
     */
    public static double distance(Node a, Node b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return (isDepot ? "D" : "C") + id;
    }
}