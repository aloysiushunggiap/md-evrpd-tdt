import algorithm.ConstraintChecker;
import algorithm.ICGA;
import model.Solution;
import util.DataLoader;

import java.io.File;
import java.util.Locale;

public class Main {

    private static final String DEFAULT_DATA_DIR =
            "D:\\Nghien cuu thuat toan\\huyrebuild\\src\\data";

    public static void main(String[] args) {

        System.out.println("=== MD-EVRPD-TDT Solver ===");

        /*
         * Cách chạy:
         * - Không truyền argument: chạy p01
         * - Truyền p04: chạy data/p04
         * - Truyền full path: chạy đúng path đó
         */
        String instanceArg = args.length > 0 ? args[0].trim() : "p03";

        String path;
        File directFile = new File(instanceArg);

        if (directFile.exists()) {
            path = directFile.getAbsolutePath();
        } else {
            path = DEFAULT_DATA_DIR + File.separator + instanceArg;
        }

        System.out.println("Data file   : " + path);

        // ===== Load data =====
        DataLoader.load(path);

        // ===== Solve =====
        long start = System.currentTimeMillis();

        Solution best = ICGA.solve();

        long end = System.currentTimeMillis();

        // ===== Final result =====
        System.out.println("\n========== FINAL SOLUTION ==========");
        System.out.printf(Locale.US,
                "Cost = %.4f | Penalty = %.4f | Fitness = %.4f | Feasible = %b%n",
                best.totalCost,
                best.totalPenalty,
                best.totalCost + best.totalPenalty,
                best.feasible
        );

        System.out.println("\n===== STRUCTURE =====");
        System.out.println("EV routes      : " + best.totalEVs());
        System.out.println("Used drones    : " + best.totalDrones());
        System.out.println("EV served      : " + best.totalCustomersServedByEV());
        System.out.println("Drone served   : " + best.totalCustomersServedByDrone());

        System.out.println("\n===== ROUTES =====");
        System.out.println(best);
        System.out.println("Violation = " + ConstraintChecker.checkAll(best));

        System.out.printf(Locale.US,
                "%nRuntime: %.2f seconds%n",
                (end - start) / 1000.0
        );
    }
}