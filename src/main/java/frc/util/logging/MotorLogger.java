package frc.util.logging;

import java.util.ArrayList;
import java.util.List;

public class MotorLogger implements Runnable {
    private final ArrayList<double[]> data;
    private final Loggable device;
    private final long initialTime;

    public MotorLogger(Loggable device) {
        this.device = device;
        this.data = new ArrayList<double[]>(20);
        this.initialTime = System.currentTimeMillis();
    }

    public void run() {
        double[] deviceData = device.getLogOutput();
        double[] entry = new double[deviceData.length+1];
        entry[0] = (System.currentTimeMillis() - initialTime) / 1000.0;
        data.add(entry);
    }

    public List<double[]> getData() {
        return this.data;
    }

    public void saveDataToCSV(String filename) {
        CSVSaver.saveFile(filename, this.data);
    }
}
