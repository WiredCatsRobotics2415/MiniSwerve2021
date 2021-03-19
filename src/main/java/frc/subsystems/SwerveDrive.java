package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.util.Vector2D;

public class SwerveDrive {
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final AHRS navX;

    private final double maxModuleRadius;
    //private double prevAngle;
    //private long prevTime;
    

    public static final double TURNING_KP = 0.015, TURNING_KD = 0.0, MAX_ADJUSTMENT = 0.3;

    public SwerveDrive() {
        this(false, false);
    }

    public SwerveDrive(boolean tuning, boolean logging) {
        this.frontLeftModule = new SwerveModule(RobotMap.FRONT_LEFT_SWERVE_DRIVE, RobotMap.FRONT_LEFT_SWERVE_AZIMUTH,
                RobotMap.FRONT_LEFT_SWERVE_AZIMUTH_REV, RobotMap.FRONT_LEFT_AZIMUTH_ENCODER,
                RobotMap.FRONT_LEFT_MODULE_X, RobotMap.FRONT_LEFT_MODULE_Y, Constants.FRONT_LEFT_AZIMUTH_PID,
                Constants.FRONT_LEFT_AZIMUTH_ENCODER_OFFSET, RobotMap.FRONT_LEFT_AZIMUTH_ENCODER_REV, tuning, logging, Constants.FRONT_LEFT_MODULE_NAME);
        this.frontRightModule = new SwerveModule(RobotMap.FRONT_RIGHT_SWERVE_DRIVE, RobotMap.FRONT_RIGHT_SWERVE_AZIMUTH,
                RobotMap.FRONT_RIGHT_SWERVE_AZIMUTH_REV, RobotMap.FRONT_RIGHT_AZIMUTH_ENCODER,
                RobotMap.FRONT_RIGHT_MODULE_X, RobotMap.FRONT_RIGHT_MODULE_Y, Constants.FRONT_RIGHT_AZIMUTH_PID,
                Constants.FRONT_RIGHT_AZIMUTH_ENCODER_OFFSET, RobotMap.FRONT_RIGHT_AZIMUTH_ENCODER_REV, tuning, logging, Constants.FRONT_RIGHT_MODULE_NAME);
        this.backLeftModule = new SwerveModule(RobotMap.BACK_LEFT_SWERVE_DRIVE, RobotMap.BACK_LEFT_SWERVE_AZIMUTH,
                RobotMap.BACK_LEFT_SWERVE_AZIMUTH_REV, RobotMap.BACK_LEFT_AZIMUTH_ENCODER, RobotMap.BACK_LEFT_MODULE_X,
                RobotMap.BACK_LEFT_MODULE_Y, Constants.BACK_LEFT_AZIMUTH_PID,
                Constants.BACK_LEFT_AZIMUTH_ENCODER_OFFSET, RobotMap.BACK_LEFT_AZIMUTH_ENCODER_REV, tuning, logging, Constants.BACK_LEFT_MODULE_NAME);
        this.backRightModule = new SwerveModule(RobotMap.BACK_RIGHT_SWERVE_DRIVE, RobotMap.BACK_RIGHT_SWERVE_AZIMUTH,
                RobotMap.BACK_RIGHT_SWERVE_AZIMUTH_REV, RobotMap.BACK_RIGHT_AZIMUTH_ENCODER,
                RobotMap.BACK_RIGHT_MODULE_X, RobotMap.BACK_RIGHT_MODULE_Y, Constants.BACK_RIGHT_AZIMUTH_PID,
                Constants.BACK_RIGHT_AZIMUTH_ENCODER_OFFSET, RobotMap.BACK_RIGHT_AZIMUTH_ENCODER_REV, tuning, logging, Constants.BACK_RIGHT_MODULE_NAME);

        this.navX = new AHRS(Port.kMXP);
        this.maxModuleRadius = Math.max(Math.max(this.frontLeftModule.getRadius(), this.frontRightModule.getRadius()),
                Math.max(this.backLeftModule.getRadius(), this.backRightModule.getRadius()));
    }

    public void zeroYaw() {
        navX.zeroYaw();
    }

    public void drive(double x, double y, double r) {
        Vector2D strafeVector = Vector2D.vectorFromRectForm(x, y);
        double yaw = navX.getYaw();
        strafeVector = strafeVector.rotate(yaw, true);
        /*if (r == 0 && strafeVector.getLength() != 0) {
            r = (prevAngle - yaw) * TURNING_KP
                    + ((prevAngle - yaw) / (System.currentTimeMillis() - prevTime)) * TURNING_KD;
            if (Math.abs(r) > MAX_ADJUSTMENT) {
                r = Math.copySign(MAX_ADJUSTMENT, r);
            }
        } else {
            prevAngle = yaw;
        }*/
        Vector2D fLVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, frontLeftModule));
        Vector2D fRVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, frontRightModule));
        Vector2D bLVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, backLeftModule));
        Vector2D bRVector = Vector2D.addVectors(strafeVector, getTurnAngleVector(r, backRightModule));
        double maxLength = Math.max(Math.max(fRVector.getLength(), fLVector.getLength()), Math.max(bRVector.getLength(), bLVector.getLength()));
        if (maxLength > 1) {
            fLVector = fLVector.scale(1 / maxLength);
            fRVector = fRVector.scale(1 / maxLength);
            bLVector = bLVector.scale(1 / maxLength);
            bRVector = bRVector.scale(1 / maxLength);
        }
        frontLeftModule.setVector(fLVector);
        frontRightModule.setVector(fRVector);
        backLeftModule.setVector(bLVector);
        backRightModule.setVector(bRVector);
    }

    public void printEncoderValues() {
        frontLeftModule.printAzimuthEncoderValue();
        frontRightModule.printAzimuthEncoderValue();
        backLeftModule.printAzimuthEncoderValue();
        backRightModule.printAzimuthEncoderValue();
    }

    public void printModuleEncoders(short moduleNum) {
        SwerveModule module = this.getModule(moduleNum);
        module.printAzimuthEncoderValue();
        module.printAzimuthTalonEncoderValue();
    }

    public void setAngle(double degrees) {
        frontLeftModule.setAngleSimple(degrees);
        frontRightModule.setAngleSimple(degrees);
        backLeftModule.setAngleSimple(degrees);
        backRightModule.setAngleSimple(degrees);
    }

    public void zeroEncoders() {
        frontLeftModule.zeroEncoder();
        frontRightModule.zeroEncoder();
        backLeftModule.zeroEncoder();
        backRightModule.zeroEncoder();
    }

    private Vector2D getTurnAngleVector(double r, SwerveModule module) {
        double angle;
        angle = Math.atan2(module.getPositionY()-RobotMap.CENTER_OF_MASS_Y,
                module.getPositionX() - RobotMap.CENTER_OF_MASS_X)-Math.PI/2;
        return new Vector2D(r * module.getRadius() / maxModuleRadius, angle); //changed from min radius so I need to check
    }

    private SwerveModule getModule(int moduleNum) {
        SwerveModule module;
        switch(moduleNum) {
            case 0:
                module = this.frontRightModule;
                break;
            case 1:
                module = this.frontLeftModule;
                break;
            case 2:
                module = this.backLeftModule;
                break;
            case 3:
                module = this.backRightModule;
                break;
            default:
                module = this.frontRightModule;
                break;
        }
        return module;
    }

    public void saveLog() {
        frontLeftModule.saveLog();
        frontRightModule.saveLog();
        backLeftModule.saveLog();
        backRightModule.saveLog();
    }
}
