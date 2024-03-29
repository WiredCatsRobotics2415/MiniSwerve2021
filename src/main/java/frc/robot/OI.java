package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private XboxController controller;

    public OI() {
        this(0);
    }

    public OI(int port) {
        this.controller = new XboxController(port);
    }

    public double getX() {
        double val = this.controller.getRawAxis(0);
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public double getY() {
        double val = this.controller.getRawAxis(1) * -1;
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public double getRotation() {
        double val = this.controller.getRawAxis(2);
        if (Math.abs(val) < Constants.DEADBAND)
            return 0;
        return val;
    }

    public boolean getIntakeToggle() {
        return this.controller.getRawButtonPressed(3); // right bumper
    }

    public boolean getIntakeExtensionToggle() {
        return this.controller.getRawButtonPressed(4); // left bumper
    }

    public boolean getCompressorToggle() {
        return this.controller.getRawButtonPressed(2); // x button
    }

    public boolean getRightTurningToggle() {
        return this.controller.getRawButtonPressed(6); // back right
    }

    public boolean getLeftTurningToggle() {
        return this.controller.getRawButtonPressed(5); // back right
    }

    public boolean getRawButtonPressed(int button) {
        return this.controller.getRawButtonPressed(button);
    }
}