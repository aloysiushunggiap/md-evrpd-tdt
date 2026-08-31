import model.Solution;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class CsvExporter {

    /**
     * @param solver               "icga" or "alns" -- without this column ICGA and ALNS rows
     *                             are indistinguishable in the CSV
     * @param objectiveEvaluations full Stage 3 evaluations, the only unit in which the two
     *                             solvers can be compared fairly (see Decoder)
     */
    public static void appendResult(
            String instance,
            String solver,
            Solution best,
            long runtimeMillis,
            long objectiveEvaluations,
            String fileName
    ) {

        try {

            File file = new File(fileName);

            boolean writeHeader = !file.exists();

            FileWriter writer = new FileWriter(file, true);

            if (writeHeader) {
                writer.write(
                        "Instance;Solver;Cost;Penalty;Fitness;Feasible;"
                                + "EV_Routes;Used_Drones;"
                                + "EV_Served;Drone_Served;"
                                + "Runtime_Seconds;Objective_Evaluations\n"
                );
            }

            writer.write(
                    String.format(
                            Locale.US,
                            "%s;%s;%.4f;%.4f;%.4f;%b;%d;%d;%d;%d;%.4f;%d\n",
                            instance,
                            solver,
                            best.totalCost,
                            best.totalPenalty,
                            best.totalCost + best.totalPenalty,
                            best.feasible,
                            best.totalEVs(),
                            best.totalDrones(),
                            best.totalCustomersServedByEV(),
                            best.totalCustomersServedByDrone(),
                            runtimeMillis / 1000.0,
                            objectiveEvaluations
                    ).replace('.',',')
            );

            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}