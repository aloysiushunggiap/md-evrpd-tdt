import model.Solution;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class CsvExporter {

    public static void appendResult(
            String instance,
            Solution best,
            long runtimeMillis,
            String fileName
    ) {

        try {

            File file = new File(fileName);

            boolean writeHeader = !file.exists();

            FileWriter writer = new FileWriter(file, true);

            if (writeHeader) {
                writer.write(
                        "Instance;Cost;Penalty;Fitness;Feasible;"
                                + "EV_Routes;Used_Drones;"
                                + "EV_Served;Drone_Served;"
                                + "Runtime_Seconds\n"
                );
            }

            writer.write(
                    String.format(
                            Locale.US,
                            "%s;%.4f;%.4f;%.4f;%b;%d;%d;%d;%d;%.4f\n",
                            instance,
                            best.totalCost,
                            best.totalPenalty,
                            best.totalCost + best.totalPenalty,
                            best.feasible,
                            best.totalEVs(),
                            best.totalDrones(),
                            best.totalCustomersServedByEV(),
                            best.totalCustomersServedByDrone(),
                            runtimeMillis / 1000.0
                    ).replace('.',',')
            );

            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}