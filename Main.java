import algorithm.ALNS;
import algorithm.ConstraintChecker;
import algorithm.Decoder;
import algorithm.ICGA;
import model.Solution;
import util.DataLoader;

import java.io.File;
import java.util.Locale;

public class Main {

    private static final String DEFAULT_DATA_DIR = "D:\\Nghien cuu thuat toan\\huyrebuild\\src\\data\\";

    public static void main(String[] args) {

        String instanceArg = "p06";
        boolean exportCsv = false;
        String solver = "alns";

        for (String arg : args) {
            if ("--csv".equalsIgnoreCase(arg))   exportCsv = true;
            else if ("--alns".equalsIgnoreCase(arg)) solver = "alns";
            else if ("--icga".equalsIgnoreCase(arg)) solver = "icga";
            else if (!arg.startsWith("--"))      instanceArg = arg.trim();
        }

        System.out.println("=== MD-EVRPD-TDT Solver (" + solver.toUpperCase() + ") ===");

        String path;

        File directFile = new File(instanceArg);

        if (directFile.exists()) {
            path = directFile.getAbsolutePath();
        } else {
            path = DEFAULT_DATA_DIR
                    + File.separator
                    + instanceArg;
        }

        System.out.println("Data file : " + path);

        // ===== Load Data =====
        DataLoader.load(path);

        // ===== Start =====
        long start = System.currentTimeMillis();

        Solution best = solver.equals("alns") ? ALNS.solve() : ICGA.solve();

        long end = System.currentTimeMillis();

        // ===== Print Result =====
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

        System.out.printf(Locale.US, "%nRuntime: %.2f seconds%n", (end - start) / 1000.0);

        // Full Stage 3 evaluations -- the only unit comparable across ICGA and ALNS.
        System.out.println("Objective evaluations: " + Decoder.objectiveEvaluations);

        // ===== Export CSV =====
        String instanceName = new File(path).getName();

        if (exportCsv) {
            CsvExporter.appendResult(instanceName, solver, best, end - start,
                    Decoder.objectiveEvaluations, "result.csv");
        }
    }
}
