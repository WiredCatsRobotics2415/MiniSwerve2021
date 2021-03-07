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
        return this.controller.getRawAxis(0);
    }

    public double getY() {
        return this.controller.getRawAxis(1)*-1;
    }

    public double getRotation() {
        return this.controller.getRawAxis(4);
    }

    public boolean getIntakeToggle() {
        return this.controller.getRawButtonPressed(0);
    }

    public boolean getIntakeExtensionToggle() {
        return this.controller.getRawButtonPressed(0);
    }

    public boolean getCompressingToggle() {
        return this.controller.getRawButtonPressed(0);
    }
}