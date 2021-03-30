package frc.util;

import frc.robot.RobotMap;
import frc.subsystems.SwerveDrive;
import frc.subsystems.SwerveModule;
import frc.util.Vector2D;

public class SwerveOdometry implements Runnable {
    private Vector2D position;
    private double theta;

    private final double thetaOffset;

    private double frLastValue,flLastValue,blLastValue,brLastValue;

    private SwerveModule fr,fl,bl,br;
    private SwerveDrive swerveDrive;

    public SwerveOdometry(SwerveDrive swerveDrive) {
        this(0,0,0,swerveDrive);
    }

    public SwerveOdometry(double x, double y, double theta, SwerveDrive swerveDrive) {
        this.position = Vector2D.vectorFromRectForm(x,y);
        this.theta = theta;
        this.fr = swerveDrive.getModule((short)0);
        this.fl = swerveDrive.getModule((short)1);
        this.bl = swerveDrive.getModule((short)2);
        this.br = swerveDrive.getModule((short)3);
        this.swerveDrive = swerveDrive;
        this.thetaOffset = -swerveDrive.getYaw()+this.theta;
        this.frLastValue = fr.getDrivePosition();
        this.flLastValue = fl.getDrivePosition();
        this.blLastValue = bl.getDrivePosition();
        this.brLastValue = br.getDrivePosition();
    }

    private Vector2D getTurningUnitVector(SwerveModule module) {
        Vector2D v = Vector2D.vectorFromRectForm(module.positionX-RobotMap.CENTER_OF_MASS_X, module.positionY-RobotMap.CENTER_OF_MASS_Y);
        v = v.scale(1/v.getLength()); //reduce to 0
        return v.rotate(90, true);
    }

    public void iterate() {
        this.theta = swerveDrive.getYaw()+this.thetaOffset;
        Vector2D frChange = new Vector2D(this.fr.getDrivePosition()-this.frLastValue,this.fr.getAzimuthAngle(),true);
        Vector2D flChange = new Vector2D(this.fl.getDrivePosition()-this.flLastValue,this.fl.getAzimuthAngle(),true);
        Vector2D blChange = new Vector2D(this.bl.getDrivePosition()-this.blLastValue,this.bl.getAzimuthAngle(),true);
        Vector2D brChange = new Vector2D(this.br.getDrivePosition()-this.brLastValue,this.br.getAzimuthAngle(),true);
        Vector2D translationVector = Vector2D.addVectors(frChange,flChange,blChange,brChange).scale(0.25);
        double thetaChange = (frChange.dot(getTurningUnitVector(fr))+flChange.dot(getTurningUnitVector(fl))+blChange.dot(getTurningUnitVector(bl))+brChange.dot(getTurningUnitVector(br)))/4.0;
        //might use thetaChange later
        this.position = this.position.add(translationVector);
    }

    public void run() {
        iterate();
    }

    public double getX() {
        return this.position.getX();
    }
    
    public double getY() {
        return this.position.getY();
    }

    public Vector2D getPosition() {
        return this.position;
    }

    public double getTheta() {
        return this.theta;
    }
}