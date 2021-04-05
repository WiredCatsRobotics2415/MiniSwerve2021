package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.util.logging.SwerveModuleLogger.SwerveModuleLoggerMode;
import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;

public class Constants {
    public static final int kCanTimeoutMs = 30;

    public static final PIDValue FRONT_LEFT_AZIMUTH_PID = new PIDValue(31, 0.0, 70);
    public static final PIDValue FRONT_RIGHT_AZIMUTH_PID = new PIDValue(17, 0.0, 50);
    public static final PIDValue BACK_LEFT_AZIMUTH_PID = new PIDValue(22, 0.0, 66);
    public static final PIDValue BACK_RIGHT_AZIMUTH_PID = new PIDValue(30, 0.0, 55);

    public static final PIDFValue FRONT_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue FRONT_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_LEFT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);
    public static final PIDFValue BACK_RIGHT_VEL_DRIVE_PIDF = new PIDFValue(0.00, 0.0, 0.0, 0.0);

    public static final PIDValue DRIVE_DISTANCE_PID = new PIDValue(10, 0.0, 0.0);

    public static final double KS = 0.164, KV = 0.97, KA = 0.1;// 1.0;

    public static final double FRONT_LEFT_AZIMUTH_ENCODER_OFFSET = 304.56 - 180.33;// 304-180; // offset by 90
    public static final double FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET = 163.28 + 180;// 165.24-180;//146.5+180;//5.976+180;
    public static final double MOTORMIN = 0.05;
    public static final double BACK_LEFT_AZIMUTH_ENCODER_OFFSET = 218;// 219.7;
    public static final double BACK_RIGHT_AZIMUTH_ENCODER_OFFSET = 211 + 2;// 211.6;
    public static final double DEADBAND = 0.1;

    public static NeutralMode DRIVE_BREAK_MODE = NeutralMode.Brake;

    // tuning
    public static final boolean SWERVE_TUNING = false;
    public static final String FRONT_LEFT_MODULE_NAME = "FL";
    public static final String FRONT_RIGHT_MODULE_NAME = "FR";
    public static final String BACK_LEFT_MODULE_NAME = "BL";
    public static final String BACK_RIGHT_MODULE_NAME = "BR";

    // logging
    public static final boolean SWERVE_LOGGING = true;
    public static final SwerveModuleLoggerMode SWERVE_LOGGING_MODE = SwerveModuleLoggerMode.DRIVE_WCURRENT;
}
