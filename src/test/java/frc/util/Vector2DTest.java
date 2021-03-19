package frc.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class Vector2DTest {
    @Test
    public void radianOverflow() {
        Vector2D vector2d = new Vector2D(1, 7 * Math.PI);
        assertEquals(vector2d.getAngleRad(), Math.PI, 0.0001);
    }

    @Test
    public void radianUnderflow() {
        Vector2D vector2d = new Vector2D(1, -7 * Math.PI);
        assertEquals(vector2d.getAngleRad(), Math.PI, 0.0001);
    }

    @Test
    public void degreeOverflow() {
        Vector2D vector2d = new Vector2D(1, 3601, true);
        assertEquals(vector2d.getAngleDeg(), 1, 0.0001);
    }

    @Test
    public void degreeUnderflow() {
        Vector2D vector2d = new Vector2D(1, -3601, true);
        assertEquals(vector2d.getAngleDeg(), 359, 0.0001);
    }

    @Test
    public void rectFormXY() {
        Vector2D vector2D = Vector2D.vectorFromRectForm(3, 5);
        assertEquals(vector2D.getX(), 3, 0.0001);
        assertEquals(vector2D.getY(), 5, 0.0001);
    }
}