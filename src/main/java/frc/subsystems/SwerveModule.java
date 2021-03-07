package frc.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.util.Vector2D;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.PIDTunable;
import frc.util.pid.PIDTuner;
import frc.util.pid.PIDValue;
import frc.util.pid.SparkPositionControllerPWMEncoder;
import frc.util.pid.TalonFxTunable;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveModule {
    private static final double AZIMUTH_POSITION_TOLERANCE = 1.5, AZIMUTH_VELOCITY_TOLERANCE = 1000000000000.0;

    private final TalonFX driveMotor, azimuthMotor; // motors

    private final PWMAbsoluteEncoder azimuthEncoder; // absolute encoder

    private TalonFxTunable azimuthController;

    private final double positionX, positionY, radius;
    private double prevAzimuthSetpoint;
    private int turns;
    private boolean azimuthReversed;

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.azimuthMotor = new TalonFX(azimuthMotorID);

        this.azimuthEncoder = new PWMAbsoluteEncoder(azimuthEncoderChannel, azimuthEncoderOffset, azimuthReversed);

        this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        // Missing current limit
        this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.azimuthMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);

        this.azimuthMotor.configSelectedFeedbackCoefficient(360.0/(2048*56.0/3.0), 0, Constants.kCanTimeoutMs);

        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.azimuthMotor.setNeutralMode(NeutralMode.Brake);
        this.driveMotor.setInverted(false);
        this.azimuthMotor.setInverted(false);

        this.azimuthMotor.configNominalOutputForward(0, Constants.kCanTimeoutMs);
		this.azimuthMotor.configNominalOutputReverse(0, Constants.kCanTimeoutMs);
		this.azimuthMotor.configPeakOutputForward(1, Constants.kCanTimeoutMs);
        this.azimuthMotor.configPeakOutputReverse(-1, Constants.kCanTimeoutMs);

        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));
        this.azimuthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, .1));
        
        this.azimuthMotor.config_kP(0,pidValues.getKP(), Constants.kCanTimeoutMs);
        this.azimuthMotor.config_kI(0,pidValues.getKI(), Constants.kCanTimeoutMs);
        this.azimuthMotor.config_kD(0,pidValues.getKD(), Constants.kCanTimeoutMs);
        
        this.azimuthMotor.configAllowableClosedloopError(0, 2.0, Constants.kCanTimeoutMs);
        
        this.azimuthMotor.configIntegratedSensorOffset(this.azimuthEncoder.getRotationDegrees(), Constants.kCanTimeoutMs);

        this.positionX = positionX;
        this.positionY = positionY;
        this.radius = Math.hypot(this.getPositionX() - RobotMap.CENTER_OF_MASS_X,
                this.getPositionY() - RobotMap.CENTER_OF_MASS_Y);

        this.azimuthReversed = false;

        this.azimuthController = new TalonFxTunable(this.azimuthMotor, pidValues);

        this.azimuthController.setSetpoint(this.azimuthEncoder.getRotationDegrees());
        this.prevAzimuthSetpoint = this.azimuthEncoder.getRotationDegrees();
        this.turns = 0;
    }

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed, boolean azimuthTuning, String name) {
        this(driveMotorID, azimuthMotorID, azimuthRev, azimuthEncoderChannel, positionX, positionY, pidValues,
                azimuthEncoderOffset, azimuthEncoderReversed);
        if(azimuthTuning) {
            this.azimuthController.enableTuning(name);
        }
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

    public void setAngle(double degrees) { // must be called every 20ms
        double currentDirection = azimuthEncoder.getRotationDegrees();
        if (Math.abs(degrees - currentDirection) > 90 && Math.abs(degrees - currentDirection) < 270) {
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
        azimuthController.setSetpoint(degrees+(turns*360.0));
        azimuthController.run();
        this.prevAzimuthSetpoint = degrees;
    }

    public void setPercentSpeed(double percent) {
        this.driveMotor.set(TalonFXControlMode.PercentOutput, percent);
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
}
