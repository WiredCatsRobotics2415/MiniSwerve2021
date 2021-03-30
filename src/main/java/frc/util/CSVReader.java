package frc.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class CSVReader {
    private ArrayList<double[]> data;

    public CSVReader(String filename) {
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                double[] doubleValues = new double[values.length];
                try {
                    for(int i = 0; i < values.length; i++) {
                        doubleValues[i] = Double.parseDouble(values[i]);
                    }
                    this.data.add(doubleValues);
                } catch(NumberFormatException e) {
                    System.out.println(filename + " includes a non-Number");
                }
            }
        } catch(IOException e) {
            System.out.println(e);
        } catch(NumberFormatException e) {
            System.out.println(filename + " includes a non-Number");
        }
    }

    public double[][] getValues() {
        double[][] arr = new double[this.data.size()][];
        for(int i = 0; i < arr.length; i++) {
            arr[i] = new double[this.data.get(i).length];
            for(int j = 0; j < arr[i].length; j++) {
                arr[i][j] = this.data.get(i)[j];
            }
        }
        return arr;
    }
}