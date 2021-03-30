package frc.util;

import frc.subsystems.SwerveDrive;
import frc.util.pid.PIDValue;
import frc.util.pid.TunablePIDController;

public class PathFollowerController implements Runnable {
    private final double kS, kV, kA;
    private final TunablePIDController distancePIDController;

    private double lookAheadTime;
    private MotionState[] trajectory;
    private SwerveDrive swerveDrive;
    private SwerveOdometry odometry;
    private long startTime;
    private int index;

    //trajectory should be in feet and degrees
    public PathFollowerController(SwerveDrive swerveDrive,MotionState[] trajectory, double kS, double kV, double kA, double lookAheadTime, PIDValue distancePID) {
        this.trajectory = trajectory.clone();
        if(this.trajectory.length == 0) {
            System.err.println("empty trajectory");
        } else {
            this.odometry = new SwerveOdometry(this.trajectory[0].x, this.trajectory[0].y, this.trajectory[0].theta, swerveDrive);
        }
        this.distancePIDController = new TunablePIDController(distancePID);
        this.distancePIDController.setSetpoint(0);
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.lookAheadTime = lookAheadTime;
        this.startTime = System.currentTimeMillis();
        this.index = 0;
    }

    public PathFollowerController(SwerveDrive swerveDrive,double[][] trajectory, double kS, double kV, double kA, double lookAheadTime, PIDValue distancePID) {
        this(swerveDrive,convertFromArray(trajectory),kS,kV,kA,lookAheadTime, distancePID);
    }

    public void start() {
        this.startTime = System.currentTimeMillis();
        run();
    }

    public void run() {
        odometry.iterate();
        long time = System.currentTimeMillis();
        this.index = getClosestStateIndex(time);
        MotionState currentState = this.trajectory[this.index];
        int lookAheadIndex = getClosestStateIndex(time+(long)((this.lookAheadTime)*1000));
        MotionState lookAheadState = this.trajectory[lookAheadIndex];
        double feedForwardVoltage = this.kS*Math.signum(currentState.velocity)+this.kV*currentState.velocity+this.kA*currentState.accel;
        double direction = Vector2D.vectorFromRectForm(lookAheadState.x-odometry.getX(), lookAheadState.y-odometry.getY()).getAngleDeg(); //in degrees
        Vector2D velocity = new Vector2D(currentState.velocity, direction, true);
        double distanceError = Math.hypot(odometry.getX()-currentState.x,odometry.getY()-currentState.y)* //distance from target with pos and negative adjusted
            Math.signum(Vector2D.vectorFromRectForm(odometry.getX()-currentState.x,odometry.getY()-currentState.y).dot(new Vector2D(1,currentState.direction,true)));
        feedForwardVoltage += distancePIDController.calculate(distanceError, 0); //
        this.swerveDrive.veloictyDriveWithFF(velocity.getX(), velocity.getY(), currentState.omega, feedForwardVoltage);
    }

    private int getClosestStateIndex(long time) { //in millis
        int closestIndex = this.index;
        long timeError = Long.MAX_VALUE;
        for(int i = closestIndex; i < this.trajectory.length; i++) { //find the closest time in the trajectory
            if(Math.abs(this.trajectory[i].time-time) > timeError) {
                closestIndex = i;
                break;
            }
        }
        return closestIndex;
    }

    private static MotionState[] convertFromArray(double[][] array) {
        return convertFromArray(array,0,1, 2, 3, 4, 5, 6, 7, 8);
    }
    private static MotionState[] convertFromArray(double[][] array,int timeIndex, int xIndex, int yIndex, int thetaIndex, int velocityIndex, int directionIndex, int omegaIndex, int accelIndex, int alphaIndex) {
        MotionState[] trajectory = new MotionState[array.length];
        for(int i = 0; i < trajectory.length; i++) {
            try {
                trajectory[i] = new MotionState(array[i][timeIndex], array[i][xIndex], array[i][yIndex], array[i][thetaIndex], array[i][velocityIndex], array[i][directionIndex], array[i][omegaIndex], array[i][accelIndex], array[i][alphaIndex]);
            } catch(ArrayIndexOutOfBoundsException e) {
                System.err.println("inputed trajectory does not meet specificied array config");
            }
        }
        return trajectory;
    }
}