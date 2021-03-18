package frc.robot;

import frc.util.pid.PIDValue;

public class Constants {
    public static final int kCanTimeoutMs = 30;

    public static final PIDValue FRONT_LEFT_AZIMUTH_PID = new PIDValue(31, 0.0, 70);
    public static final PIDValue FRONT_RIGHT_AZIMUTH_PID = new PIDValue(20, 0.0, 60);
    public static final PIDValue BACK_LEFT_AZIMUTH_PID = new PIDValue(22, 0.0, 66);
    public static final PIDValue BACK_RIGHT_AZIMUTH_PID = new PIDValue(30, 0.0, 55);

    public static final double FRONT_LEFT_AZIMUTH_ENCODER_OFFSET = 304-180; // offset by 90
    public static final double FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET = 5.976+180;
    public static final double MOTORMIN = 0.05;
    public static final double BACK_LEFT_AZIMUTH_ENCODER_OFFSET = 219.7;
    public static final double BACK_RIGHT_AZIMUTH_ENCODER_OFFSET = 211.6;
    public static final double DEADBAND = 0.1;
}
