package frc.util;

public class MotionState {
    public final double time,x,y,theta,velocity,direction,omega,accel,alpha;

    public MotionState(double time,double x, double y, double theta, double velocity, double direction, double omega, double accel, double alpha) {
        this.time = time;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.velocity = velocity;
        this.direction = direction;
        this.omega = omega;
        this.accel = accel;
        this.alpha = alpha;
    }

    @Override
    public String toString() {
        return "time:"+time+" x:"+x+" y:"+y+" theta:"+theta+" vel:"+velocity+" direction:"+direction+" omega:"+omega+" accel:"+accel+" alpha"+alpha;
    }
}