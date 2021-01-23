package frc.robot;

import frc.util.pid.PIDValue;

public class Constants {
    public static final int kCanTimeoutMs = 30;

    public static final PIDValue FRONT_LEFT_AZIMUTH_PID = new PIDValue(0.01, 0.0, 0.00015);
    public static final PIDValue FRONT_RIGHT_AZIMUTH_PID = new PIDValue(0.015, 0.0, 0.00022);
    public static final PIDValue BACK_LEFT_AZIMUTH_PID = new PIDValue(0.0125, 0.0, 0.0002);
    public static final PIDValue BACK_RIGHT_AZIMUTH_PID = new PIDValue(0.011, 0.0, 0.0002);

    public static final double FRONT_LEFT_AZIMUTH_ENCODER_OFFSET = 167.4 - 90.0; // offset by 90
    public static final double FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET = 53.2 + 225.5 - 180.0;
    public static final double MOTORMIN = 0.03;
    public static final double BACK_LEFT_AZIMUTH_ENCODER_OFFSET = 306.6 + 107.5;
    public static final double BACK_RIGHT_AZIMUTH_ENCODER_OFFSET = 201.43;
    public static final double DEADBAND = 0.05;
}
