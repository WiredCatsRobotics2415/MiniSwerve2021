package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;

public class Intake {
    private DoubleSolenoid rightSolenoid, leftSolenoid;
    private CANSparkMax intakeMotor;
    
    private boolean extended;
    private boolean intaking;
    
    public Intake() {
        this.leftSolenoid = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.INTAKE_LEFT_FORWARD_SOLENNOID, RobotMap.INTAKE_LEFT_REVERSE_SOLENNOID);
        this.rightSolenoid = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.INTAKE_RIGHT_FORWARD_SOLENNOID, RobotMap.INTAKE_RIGHT_REVERSE_SOLENNOID);
        
        this.leftSolenoid.set(Value.kReverse);
        this.rightSolenoid.set(Value.kReverse);
        
        this.intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);

        this.intakeMotor.restoreFactoryDefaults();
        this.intakeMotor.setInverted(false);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.stopMotor();

        this.extended = false;
        this.intaking = false;
    }

    public void extend() {
        this.leftSolenoid.set(Value.kForward);
        this.rightSolenoid.set(Value.kForward);
        this.extended = true;
    }

    public void retract() {
        this.leftSolenoid.set(Value.kReverse);
        this.rightSolenoid.set(Value.kReverse);
        this.extended = false;
    }

    public void toggleExtension() {
        if(this.extended) {
            this.retract();
        } else {
            this.extend();
        }
    }

    public void intake() {
        this.intakeMotor.set(.3);
        this.intaking = true;
    }

    public void stopIntaking() {
        this.intakeMotor.set(0);
        this.intaking = false;
    }

    public void toggleIntaking() {
        if(intaking) {
            this.stopIntaking();
        } else {
            this.intake();
        }
    }
}