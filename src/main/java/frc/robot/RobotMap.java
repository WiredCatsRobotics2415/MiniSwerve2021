package frc.robot;

public class RobotMap {
    
    public static final int FRONT_LEFT_SWERVE_AZIMUTH = 24;
    public static final int FRONT_RIGHT_SWERVE_AZIMUTH = 26;
    public static final int BACK_LEFT_SWERVE_AZIMUTH = 25;
    public static final int BACK_RIGHT_SWERVE_AZIMUTH = 27;

    public static final boolean FRONT_LEFT_SWERVE_AZIMUTH_REV = false;
    public static final boolean FRONT_RIGHT_SWERVE_AZIMUTH_REV = false;
    public static final boolean BACK_LEFT_SWERVE_AZIMUTH_REV = false;
    public static final boolean BACK_RIGHT_SWERVE_AZIMUTH_REV = false;

    public static final int FRONT_LEFT_SWERVE_DRIVE = 23;
    public static final int FRONT_RIGHT_SWERVE_DRIVE = 33;
    public static final int BACK_LEFT_SWERVE_DRIVE = 22;
    public static final int BACK_RIGHT_SWERVE_DRIVE = 32;

    public static final int FRONT_LEFT_AZIMUTH_ENCODER = 3;
    public static final int FRONT_RIGHT_AZIMUTH_ENCODER = 2;
    public static final int BACK_LEFT_AZIMUTH_ENCODER = 1;
    public static final int BACK_RIGHT_AZIMUTH_ENCODER = 0;

    public static final boolean FRONT_LEFT_AZIMUTH_ENCODER_REV = true;
    public static final boolean FRONT_RIGHT_AZIMUTH_ENCODER_REV = true;
    public static final boolean BACK_LEFT_AZIMUTH_ENCODER_REV = true;
    public static final boolean BACK_RIGHT_AZIMUTH_ENCODER_REV = true;

    public static final double FRONT_LEFT_MODULE_X = 2.21, FRONT_LEFT_MODULE_Y = 2.21; // inches
    public static final double FRONT_RIGHT_MODULE_X = 27.5 - 2.21, FRONT_RIGHT_MODULE_Y = 2.21;
    public static final double BACK_LEFT_MODULE_X = 2.21, BACK_LEFT_MODULE_Y = 32 - 2.21;
    public static final double BACK_RIGHT_MODULE_X = 27.5 - 2.21, BACK_RIGHT_MODULE_Y = 32 - 2.21;

    public static final double CENTER_OF_MASS_X = 27.5 / 2;
    public static final double CENTER_OF_MASS_Y = 32.0 / 2;

}
