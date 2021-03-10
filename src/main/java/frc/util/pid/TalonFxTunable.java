package frc.util.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class TalonFxTunable implements PIDTunable{
    private final PIDValue pidValue;
    private final TalonFX talon;
    private PIDTuner tuner;


    private boolean tuning;

    public TalonFxTunable(TalonFX talon, double kP, double kI, double kD) {
        this(talon, new PIDValue(kP, kI, kD));
    }

    public TalonFxTunable(TalonFX talon, PIDValue pidValue) {
        this.talon = talon;
        this.pidValue = pidValue;
        this.setPID(pidValue.getKP(), pidValue.getKI(), pidValue.getKD());
        this.tuner = null;
        this.tuning = false;
    }

    public TalonFxTunable(TalonFX talon, PIDValue pidValue, boolean tuning, String name) {
        this(talon, pidValue);
        this.tuning = tuning;
        if(this.tuning) {
            this.tuner = new PIDTuner(this, name);
        }
    }

    public void setPID(double kP, double kI, double kD) {
        this.talon.config_kP(0, pidValue.getKP(), Constants.kCanTimeoutMs);
        this.talon.config_kI(0, pidValue.getKI(), Constants.kCanTimeoutMs);
        this.talon.config_kD(0, pidValue.getKD(), Constants.kCanTimeoutMs);
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        this.setPID(kP, kI, kD);
        this.pidValue.setPID(kP, kI, kD);
    }

    public double getKP() {
        return this.pidValue.getKP();
    }

    public double getKI() {
        return this.pidValue.getKI();
    }

    public double getKD() {
        return this.pidValue.getKD();
    }

    public double getError() {
        return talon.getClosedLoopError(0);
    }

    @Override
    public double getSetpoint() {
        return talon.getClosedLoopTarget(0);
    }

    public void enableTuning(String name) {
        if(tuning) return;
        this.tuning = true;
        this.tuner = new PIDTuner(this, name);
    }

    @Override
    public void setSetpoint(double setpoint) {
        talon.set(ControlMode.Position, setpoint);
    }

    public void run() {
        if(tuning) {
            this.tuner.update();
        }
    }
}