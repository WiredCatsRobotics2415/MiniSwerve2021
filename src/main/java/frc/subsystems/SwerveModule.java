package frc.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.util.Vector2D;
import frc.util.logging.MotorLogger;
import frc.util.logging.SwerveModuleLogger;
import frc.util.logging.SwerveModuleLogger.SwerveModuleLoggerMode;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.PIDFValue;
import frc.util.pid.PIDValue;
import frc.util.pid.TalonFxTunable;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveModule {

    private final TalonFX driveMotor, azimuthMotor; // motors

    private final PWMAbsoluteEncoder azimuthEncoder; // absolute encoder

    private final TalonFxTunable azimuthController;
    private final TalonFxTunable driveController;

    private MotorLogger logger;

    public final double positionX, positionY, radius;

    private double prevAzimuthSetpoint;
    private int turns;
    private boolean azimuthReversed;
    private String name;

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue azimuthPidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed, PIDFValue drivePIDF) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.azimuthMotor = new TalonFX(azimuthMotorID);

        this.azimuthEncoder = new PWMAbsoluteEncoder(azimuthEncoderChannel, azimuthEncoderOffset, azimuthEncoderReversed);

        this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        // Missing current limit
        this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.driveMotor.setSelectedSensorPosition(0, 0, Constants.kCanTimeoutMs);
        this.azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);

        this.azimuthMotor.configSelectedFeedbackCoefficient(360.0/(2048*56.0/3.0), 0, Constants.kCanTimeoutMs);

        this.driveMotor.setNeutralMode(Constants.DRIVE_BREAK_MODE);
        this.azimuthMotor.setNeutralMode(NeutralMode.Coast);
        this.driveMotor.setInverted(false);
        this.azimuthMotor.setInverted(false);

        this.azimuthMotor.configNominalOutputForward(0, Constants.kCanTimeoutMs);
		this.azimuthMotor.configNominalOutputReverse(0, Constants.kCanTimeoutMs);
		this.azimuthMotor.configPeakOutputForward(1, Constants.kCanTimeoutMs);
        this.azimuthMotor.configPeakOutputReverse(-1, Constants.kCanTimeoutMs);

        this.azimuthMotor.configClosedLoopPeakOutput(0, 0.8, Constants.kCanTimeoutMs);
        this.azimuthMotor.configNeutralDeadband(Constants.MOTORMIN, Constants.kCanTimeoutMs);

        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
        this.azimuthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, .1));
        
        this.azimuthMotor.config_kP(0,azimuthPidValues.getKP(), Constants.kCanTimeoutMs);
        this.azimuthMotor.config_kI(0,azimuthPidValues.getKI(), Constants.kCanTimeoutMs);
        this.azimuthMotor.config_kD(0,azimuthPidValues.getKD(), Constants.kCanTimeoutMs);

        this.driveMotor.config_kP(0,drivePIDF.getKP(), Constants.kCanTimeoutMs);
        this.driveMotor.config_kI(0,drivePIDF.getKI(), Constants.kCanTimeoutMs);
        this.driveMotor.config_kD(0,drivePIDF.getKD(), Constants.kCanTimeoutMs);
        this.driveMotor.config_kF(0,drivePIDF.getKF(), Constants.kCanTimeoutMs); //needs to be 0 if you are doing feedforward already
        
        this.azimuthMotor.configAllowableClosedloopError(0, 4.0, Constants.kCanTimeoutMs);

        this.azimuthMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, Constants.kCanTimeoutMs);

        this.positionX = positionX;
        this.positionY = positionY;
        this.radius = Math.hypot(this.getPositionX() - RobotMap.CENTER_OF_MASS_X,
                this.getPositionY() - RobotMap.CENTER_OF_MASS_Y);

        this.azimuthReversed = false;

        this.azimuthController = new TalonFxTunable(this.azimuthMotor, azimuthPidValues, ControlMode.Position);
        this.driveController = new TalonFxTunable(this.driveMotor, drivePIDF, ControlMode.Velocity);

        this.zeroEncoder();

        this.prevAzimuthSetpoint = this.azimuthEncoder.getRotationDegrees();
        this.turns = 0;

        this.logger = null;
        this.name = "";
    }

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed, PIDFValue drivePIDF, boolean azimuthTuning, boolean logging, String name) {
        this(driveMotorID, azimuthMotorID, azimuthRev, azimuthEncoderChannel, positionX, positionY, pidValues,
                azimuthEncoderOffset, azimuthEncoderReversed,drivePIDF);
        if(azimuthTuning) {
            this.azimuthController.enableTuning(name);
        }
        if(logging) {
            this.logger = new MotorLogger(new SwerveModuleLogger(this, Constants.SWERVE_LOGGING_MODE));
        }
        this.name = name;
    }

    public void setVector(Vector2D vector) {
        if (vector.getLength() != 0) {
            setAngle(vector.getAngleDeg());
        } else {
            azimuthController.run();
        }
        if (azimuthReversed) {
            setPercentSpeed(vector.getLength() * -1);
        } else {
            setPercentSpeed(vector.getLength());
        }
    }

    public void setVelocityVectorWithFF(Vector2D vector, double ff) {
        if (vector.getLength() != 0) {
            setAngle(vector.getAngleDeg());
        } else {
            azimuthController.run();
        }
        if (azimuthReversed) {
            setDriveVelocityWithFF(vector.getLength() * -1, ff);
        } else {
            setDriveVelocityWithFF(vector.getLength(), ff);
        }
    }

    public void setAngle(double degrees) { // must be called every 20ms
        //double encoderValue = azimuthMotor.getSelectedSensorPosition(0);
        //double currentDirection = Vector2D.modulus(encoderValue,360);//azimuthEncoder.getRotationDegrees();
        if (Math.abs(degrees - this.prevAzimuthSetpoint) > 90 && Math.abs(degrees - this.prevAzimuthSetpoint) < 270) {
            degrees = (degrees + 180) % 360;
            azimuthReversed = true;
        } else {
            azimuthReversed = false;
        }
        if(degrees > this.prevAzimuthSetpoint && Math.abs(degrees - this.prevAzimuthSetpoint) > 180) {
            turns--;
        } else if(degrees < this.prevAzimuthSetpoint && Math.abs(degrees - this.prevAzimuthSetpoint) > 180) {
            turns++;
        }
        this.azimuthController.setSetpoint(degrees+turns*360.0);
        this.azimuthController.run();
        this.prevAzimuthSetpoint = degrees;
    }

    public void setPercentSpeed(double percent) {
        this.driveMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void setDriveVelocityWithFF(double velocity, double ff) {
        this.driveController.setSetpointWithFF(velocity, ff);
    }

    public void zeroEncoder() {
        this.azimuthMotor.set(ControlMode.PercentOutput, 0.0);
        ErrorCode error = this.azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotationDegrees(), 0, Constants.kCanTimeoutMs);
        if(!error.equals(ErrorCode.OK)) {
            System.out.println("failed zero");
            error = this.azimuthMotor.setSelectedSensorPosition(this.azimuthEncoder.getRotationDegrees(), 0, Constants.kCanTimeoutMs);
        }
        //this.azimuthController.setSetpoint(this.azimuthEncoder.getRotationDegrees());
    }

    public void zeroDriveEncoder() {
        this.driveMotor.setSelectedSensorPosition(0.0, 0, Constants.kCanTimeoutMs);
    }

    public void setAngleSimple(double degrees) {
        azimuthController.setSetpoint(degrees);
        azimuthController.run();
    }

    public void saveLog() {
        if(this.logger != null) {
            this.logger.saveDataToCSV(this.name+"logging.csv");
        }
    }

    public double getDrivePosition() {
        return this.driveMotor.getSelectedSensorPosition(0);
    }

    public double getDriveVoltage() {
        return this.driveMotor.getMotorOutputVoltage();
    }

    public double getDriveCurrent() {
        return this.driveMotor.getStatorCurrent();
    }

    public double getAzimuthAngle() {
        return this.azimuthEncoder.getRotationDegrees();
    }

    public double getAzimuthVoltage() {
        return this.azimuthMotor.getMotorOutputVoltage();
    }

    public double getAzimuthCurrent() {
        return this.azimuthMotor.getStatorCurrent();
    }

    public double getRadius() {
        return this.radius;
    }

    public double getPositionX() {
        return positionX;
    }

    public double getPositionY() {
        return positionY;
    }

    public void log() {
        if(this.logger == null) return;
        this.logger.run();
    }

    public void printCurrent() {
        System.out.println(this.name+" "+this.driveMotor.getStatorCurrent());
    }

    public void printAzimuthTalonEncoderValue() {
        System.out.println("TalonFx x:"+this.getPositionX()+" y:"+this.getPositionY()+" Value:"+this.azimuthMotor.getSelectedSensorPosition(0) + " : "+this.azimuthMotor.getSelectedSensorPosition(0)%360);
    }

    public void printAzimuthEncoderValue() {
        System.out.println("MA3 x:"+this.getPositionX()+" y:"+this.getPositionY()+" Value:"+this.azimuthEncoder.getRotationDegrees());
    }
}
