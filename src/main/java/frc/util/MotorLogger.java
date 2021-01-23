package frc.util;

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
        double[] entry = new double[2];
        entry[0] = (System.currentTimeMillis() - initialTime) / 1000.0;
        entry[1] = device.getLogOutput();
        data.add(entry);
    }

    public List<double[]> getData() {
        return this.data;
    }
}
