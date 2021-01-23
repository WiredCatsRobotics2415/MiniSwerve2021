package frc.util;

public class Vector2D {
    private final double length, radians; // angle is degrees [0,360)

    public Vector2D(double length, double radians) {
        this.length = length;
        this.radians = reduceRadians(radians);
    }

    public Vector2D(double length, double angle, boolean degrees) {
        this(length, (degrees) ? Math.toRadians(angle) : angle);
    }

    public static Vector2D vectorFromRectForm(double x, double y) {
        double length, angle;
        length = Math.hypot(x, y);
        angle = Math.atan2(y, x);
        if (angle < 0)
            angle += Math.PI * 2;
        return new Vector2D(length, angle);
    }

    public double getLength() {
        return length;
    }

    public double getAngleRad() { // [0,pi*2)
        return radians;
    }

    public double getAngleDeg() { // [0, 360)
        return Math.toDegrees(radians);
    }

    public double getX() {
        return length * Math.cos(radians);
    }

    public double getY() {
        return length * Math.sin(radians);
    }

    public Vector2D add(Vector2D otherVector) {
        return addVectors(this, otherVector);
    }

    public Vector2D rotate(double radiansRotated) {
        return new Vector2D(this.length, this.radians + radiansRotated);
    }

    public Vector2D rotate(double angle, boolean degrees) {
        return this.rotate((degrees) ? Math.toRadians(angle) : angle);
    }

    public Vector2D scale(double scaler) {
        return new Vector2D(length * scaler, radians);
    }

    public static Vector2D addVectors(Vector2D v1, Vector2D v2) {
        return vectorFromRectForm(v1.getX() + v2.getX(), v1.getY() + v2.getY());
    }

    public static double reduceRadians(double radians) {
        radians %= Math.PI * 2;
        if (radians < 0) {
            radians += Math.PI * 2;
        }
        return radians;
    }

    @Override
    public String toString() {
        return "length: " + length + " \tangle: " + Math.toDegrees(radians);
    }
}