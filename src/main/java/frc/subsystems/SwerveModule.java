package frc.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.util.Vector2D;
import frc.util.PWMAbsoluteEncoder;
import frc.util.pid.PIDValue;
import frc.util.pid.SparkPositionControllerPWMEncoder;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveModule {
    private static final double AZIMUTH_POSITION_TOLERANCE = 1.5, AZIMUTH_VELOCITY_TOLERANCE = 1000000000000.0;

    private final TalonFX driveMotor; // motors
    private final CANSparkMax azimuthMotor;

    private final PWMAbsoluteEncoder azimuthEncoder; // absolute encoder

    private final PIDValue azimuthPIDValues;
    private SparkPositionControllerPWMEncoder azimuthController;

    private final double positionX, positionY, radius;
    private boolean azimuthReversed;

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed) {
        this.driveMotor = new TalonFX(driveMotorID);
        this.driveMotor.configFactoryDefault(Constants.kCanTimeoutMs);
        // Missing current limit
        this.driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                Constants.kCanTimeoutMs);
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.driveMotor.setInverted(false);
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, .1));

        this.azimuthMotor = new CANSparkMax(azimuthMotorID, MotorType.kBrushless);
        this.azimuthMotor.restoreFactoryDefaults();
        this.azimuthMotor.setIdleMode(IdleMode.kBrake);
        this.azimuthMotor.setInverted(azimuthRev);
        this.azimuthMotor.setSmartCurrentLimit(15);

        this.azimuthEncoder = new PWMAbsoluteEncoder(azimuthEncoderChannel, azimuthEncoderOffset,
                azimuthEncoderReversed);

        this.azimuthPIDValues = pidValues.clone();
        this.azimuthController = new SparkPositionControllerPWMEncoder(this.azimuthMotor, this.azimuthEncoder,
                this.azimuthPIDValues);
        this.azimuthController.setSetpoint(this.azimuthEncoder.getRotationDegrees());
        this.positionX = positionX;
        this.positionY = positionY;
        this.radius = Math.hypot(this.getPositionX() - RobotMap.CENTER_OF_MASS_X,
                this.getPositionY() - RobotMap.CENTER_OF_MASS_Y);

        this.azimuthReversed = false;
    }

    public SwerveModule(int driveMotorID, int azimuthMotorID, boolean azimuthRev, int azimuthEncoderChannel,
            double positionX, double positionY, PIDValue pidValues, double azimuthEncoderOffset,
            boolean azimuthEncoderReversed, boolean azimuthTuning, String name) {
        this(driveMotorID, azimuthMotorID, azimuthRev, azimuthEncoderChannel, positionX, positionY, pidValues,
                azimuthEncoderOffset, azimuthEncoderReversed);
        this.azimuthController = new SparkPositionControllerPWMEncoder(this.azimuthMotor, this.azimuthEncoder,
                this.azimuthPIDValues, azimuthTuning, name);
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
        azimuthController.setSetpoint(degrees);
        azimuthController.run();
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
