package frc.util;

import frc.subsystems.SwerveDrive;
import frc.util.logging.MotorLogger;
import frc.util.logging.OdometryLogger;
import frc.util.pid.PIDValue;
import frc.util.pid.TunablePIDController;

public class PathFollowerController implements Runnable {
    private final double kS, kV, kA;
    private final TunablePIDController xController;
    private final TunablePIDController yController;

    private MotorLogger logger;
    private double lookAheadTime;
    private MotionState[] trajectory;
    private SwerveDrive swerveDrive;
    private SwerveOdometry odometry;
    private long startTime;
    private int index;
    private boolean logged;

    //trajectory should be in feet and degrees
    public PathFollowerController(SwerveDrive swerveDrive,MotionState[] trajectory, double kS, double kV, double kA, double lookAheadTime, PIDValue distancePID, boolean logging) {
        this.trajectory = trajectory.clone();
        this.swerveDrive = swerveDrive;
        if(this.trajectory.length == 0) {
            System.err.println("empty trajectory");
        } else {
            this.odometry = new SwerveOdometry(this.trajectory[0].x, this.trajectory[0].y, this.trajectory[0].theta, swerveDrive);
        }
        this.xController = new TunablePIDController(distancePID);
        this.xController.setSetpoint(0);
        this.yController = new TunablePIDController(distancePID);
        this.yController.setSetpoint(0);
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.logged = false;
        this.lookAheadTime = lookAheadTime;
        this.startTime = System.currentTimeMillis();
        this.index = 0;
        if(logging) {
            this.logger = new MotorLogger(new OdometryLogger(this.odometry));
        } else this.logger = null;
    }

    public PathFollowerController(SwerveDrive swerveDrive,double[][] trajectory, double kS, double kV, double kA, double lookAheadTime, PIDValue distancePID) {
        this(swerveDrive,convertFromArray(trajectory),kS,kV,kA,lookAheadTime, distancePID,false);
    }

    public PathFollowerController(SwerveDrive swerveDrive,double[][] trajectory, double kS, double kV, double kA, double lookAheadTime, PIDValue distancePID, boolean logging) {
        this(swerveDrive,convertFromArray(trajectory),kS,kV,kA,lookAheadTime, distancePID, logging);
    }

    public void start() {
        this.startTime = System.currentTimeMillis();
        run();
    }

    public void run() {
        odometry.iterate();
        if(this.logger != null) this.logger.run();
        //System.out.println("x:"+odometry.getX()+" y:"+odometry.getY());
        long time = System.currentTimeMillis();
        this.index = getClosestStateIndex(time);
        if((time-startTime)/1000.0 > this.trajectory[this.trajectory.length-1].time) {
            if(!this.logged) {
                this.saveLog();
                this.logged = true;
            }
            this.swerveDrive.drive(0,0,0);
            return ;
        }
        MotionState currentState = this.trajectory[this.index];
        //int lookAheadIndex = getClosestStateIndex(time+(long)((this.lookAheadTime)*1000));
        //MotionState lookAheadState;
        //if(lookAheadIndex != -1) {
        //    lookAheadState = this.trajectory[lookAheadIndex];
        //} else {
        //    lookAheadState = this.trajectory[this.trajectory.length-1];
        //}
        double direction = currentState.direction; //in degrees
        //double direction = Vector2D.vectorFromRectForm(lookAheadState.x-odometry.getX(),lookAheadState.y-odometry.getY()).getAngleDeg();
        Vector2D velocity = new Vector2D(currentState.velocity, direction, true);
        //double distanceError = Math.hypot(odometry.getX()-currentState.x,odometry.getY()-currentState.y)* //distance from target with pos and negative adjusted
        //    Math.signum(Vector2D.vectorFromRectForm(odometry.getX()-currentState.x,odometry.getY()-currentState.y).dot(new Vector2D(1,currentState.direction,true)));
        double xError = odometry.getX()-currentState.x;
        double yError = odometry.getY()-currentState.y;
        System.out.println("odom:"+odometry.getX()+" state:"+currentState.x);
        xError = this.xController.calculate(xError);
        yError = this.xController.calculate(yError);
        System.out.println("controller:"+xError);
        velocity = velocity.add(Vector2D.vectorFromRectForm(xError, yError));
        direction = velocity.getAngleDeg();
        System.out.println(velocity);
        double feedForwardVoltage = this.kS*Math.signum(velocity.getLength())+this.kV*velocity.getLength()+this.kA*currentState.accel;
        //feedForwardVoltage += distancePIDController.calculate(distanceError, 0); //
        System.out.println(velocity);
        this.swerveDrive.velocityDriveWithFF(velocity.getX(), velocity.getY(), currentState.omega, feedForwardVoltage);
    }

    private int getClosestStateIndex(long time) { //in millis
        int closestIndex = this.index;
        double timeError = Double.MAX_VALUE;
        time -= this.startTime;
        double seconds = time/1000.0;
        if(closestIndex == this.trajectory.length-1) return -1;
        for(int i = closestIndex; i < this.trajectory.length; i++) { //find the closest time in the trajectory
            if(Math.abs(this.trajectory[i].time-seconds) > timeError) {
                closestIndex = i;
                break;
            }
            timeError = Math.abs(this.trajectory[i].time-seconds);
        }
        return closestIndex;
    }

    public void saveLog() {
        if(this.logger != null) {
            System.out.println("odom logged");
            this.logger.saveDataToCSV("odom.csv");
        }
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