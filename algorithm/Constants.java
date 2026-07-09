package algorithm;

/**
 * Hằng số dùng cho MD-EVRPD-TDT.
 *
 * Nguyên tắc:
 * - phần PAPER PARAMETERS phải bám đúng bài báo
 * - phần HEURISTIC PARAMETERS chỉ hỗ trợ giải heuristic, không thay mô hình gốc
 */
public class Constants {

    // =========================================================
    // PAPER PARAMETERS
    // =========================================================

    // ----- EV -----
    // Eq./Table tham số thực nghiệm trong paper
    public static final double EV_CAPACITY = 1000.0;     // Q_k (kg)
    public static final double EV_BATTERY = 86.0;        // B_k (kWh)
    public static final double MIN_SOC = 0.2;            // epsilon
    public static final double EV_MIN_ENERGY = MIN_SOC * EV_BATTERY;

    // ----- EV time / distance -----
    public static final double DETOUR_COEFF = 1.2;       // w
    public static final double T_START = 7.0;            // Ts
    public static final double T_END = 17.0;             // Tf
    public static final double EV_SERVICE_TIME = 3.0 / 60.0;   // ts = 3 phút

    // ----- Drone -----
    public static final double DRONE_CAPACITY = 5.0;     // Q_u (kg)
    public static final double DRONE_BATTERY = 0.27;     // B_u (kWh)
    public static final double DRONE_MIN_ENERGY = MIN_SOC * DRONE_BATTERY;
    public static final double DRONE_SPEED = 80.0;       // v_u (km/h)

    public static final double DRONE_E1 = 0.8;           // travel energy coef
    public static final double DRONE_E2 = 0.5;           // hover energy coef
    public static final double DRONE_E3 = 0.005;         // single takeoff/landing energy

    public static final double DRONE_T1 = 1.0 / 120.0;   // 0.5 phút
    public static final double DRONE_T2 = 1.0 / 60.0;    // 1 phút

    // ----- Cost -----
    public static final double EV_DISPATCH_COST = 300.0;     // c_k
    public static final double DRONE_DISPATCH_COST = 3.0;    // c_u
    public static final double ELECTRICITY_PRICE = 1.0;      // c_0

    // =========================================================
    // EV ENERGY MODEL PARAMETERS
    // =========================================================
    // Paper dùng công thức động lực học với:
    // P = 1/(3600*etaT*etaV*etaM) * v(t) * (m*g*f + C_D*A*v(t)^2/21.15 + delta*m*a)
    // Các hệ số dưới đây là phần cần thiết để EnergyUtil bám công thức paper.

    public static final double EV_MASS_EMPTY = 3000.0;   // m0 (kg)
    public static final double GRAVITY = 9.81;           // g
    public static final double ROLLING_COEFF = 0.01;     // f

    public static final double DRAG_COEFF = 0.7;         // C_D
    public static final double FRONTAL_AREA = 3.2;       // A (m^2)

    // Hiệu suất
    public static final double ETA_T = 0.90;             // transmission efficiency
    public static final double ETA_V = 0.95;             // inverter efficiency
    public static final double ETA_M = 0.85;             // motor efficiency

    // Khí động / quay
    public static final double AIR_DENSITY = 1.225;      // rho
    public static final double ROTATING_MASS_FACTOR = 1.05; // delta

    // Paper có thành phần gia tốc a trong Eq.(50)-(51).
    // Với heuristic route-level hiện tại, dùng xấp xỉ a = 0 để tránh bẻ lệch mô hình
    // bằng giả định ngoài paper.
    public static final double EV_ACCELERATION = 0.0;

    // =========================================================
    // GA/VNS search range
    // =========================================================
    public static final int POP_SIZE_SMALL = 50;
    public static final int POP_SIZE_MEDIUM = 70;
    public static final int POP_SIZE_LARGE = 100;

    public static final int MAX_GEN_SMALL = 60;
    public static final int MAX_GEN_MEDIUM = 100;
    public static final int MAX_GEN_LARGE = 150;

    public static final double CHAOTIC_R = 4.0;

    // =========================================================
    // HEURISTIC PARAMETERS
    // =========================================================

    public static final double ELITE_RATE = 0.10;
    public static final int OPERATOR_TRIALS = 5;

    // Chưa dùng để “sáng tạo thêm thuật toán”, chỉ dùng để ổn định quần thể
    public static final int DIVERSIFY_PERCENT = 25;

    // ----- Stage 1 / Stage 3 -----
    // Paper giả định mỗi depot có đủ drone.
    // Đây là upper bound heuristic, không phải ràng buộc paper.
    public static final int MAX_DEPOT_DRONES_PER_DEPOT = 4;

    public static final int STAGE2_INSERT_NEAREST_POSITIONS = 2;

    // Stage 3 search window
    public static final int STAGE3_MAX_ROUNDS = 10;
    public static final int STAGE3_LAUNCH_LOOKAHEAD = 8;
    public static final int STAGE3_SERVE_CANDIDATES = 12;
    public static final int STAGE3_RETRIEVE_CANDIDATES = 12;

    public static final double BIG_M = 1e6;

    // Bật nếu dùng trực tiếp benchmark p02/p04 gốc của MDVRP như paper.
    // Tắt nếu dữ liệu đầu vào đã được chỉnh demand sẵn.
    public static final boolean NORMALIZE_PAPER_DEMAND = true;

    // =========================================================
    // PENALTY
    // =========================================================
    // Chỉ hỗ trợ search heuristic; không thay mô hình gốc.

    public static final double PENALTY_MISSED_CUSTOMER = 5000.0;
    public static final double PENALTY_DUPLICATE_CUSTOMER = 4000.0;

    public static final double PENALTY_EV_CAPACITY = 80.0;
    public static final double PENALTY_EV_ENERGY = 120.0;
    public static final double PENALTY_TIME = 120.0;

    public static final double PENALTY_DRONE_ENERGY = 200.0;
    public static final double PENALTY_SYNC = 200.0;

    public static final double PENALTY_NODE_OPERATION = 300.0;
    public static final double PENALTY_DEPOT_BALANCE = 500.0;
    public static final double PENALTY_CONTINUITY = 600.0;
    public static final boolean ENABLE_ROUTE_MERGE = false;
    public static final int ROUTE_IMPROVE_GUARD = 2;
    public static final int MERGE_IMPROVE_GUARD = 2;

    // Dừng sớm nếu nhiều generation liên tiếp không cải thiện best solution.
    public static final int EARLY_STOP_PATIENCE = 100000;
    private Constants() {
    }
}
