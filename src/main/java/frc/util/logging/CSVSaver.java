package frc.util.logging;

import java.util.Arrays;
import java.util.List;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CSVSaver {
    public static void saveFile(String filename, List<double[]> data) {
        File file = new File(filename);
        saveFile(file, data);
    }

    public static void saveFile(File file, List<double[]> data) {
        FileWriter fr = null;
        BufferedWriter br = null;
        String line;
        try {
            fr = new FileWriter(file);
            br = new BufferedWriter(fr);
            for (double[] datum : data) {
                line = "";
                for (int j = 0; j < datum.length; j++) {
                    line += datum[j];
                    if (j < datum.length - 1) {
                        line += ",";
                    }
                }
                line += "\n";
                br.write(line);
            }
        } catch (IOException e) {
            System.out.println(e);
        } finally {
            try {
                br.close();
                fr.close();
            } catch (Exception e) {
                System.out.println(e);
            }
        }
    }
}