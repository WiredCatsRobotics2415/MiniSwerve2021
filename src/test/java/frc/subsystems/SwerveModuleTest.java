package frc.subsystems;

import frc.util.pid.PIDValue;

import static org.junit.Assert.assertEquals;

import java.util.Arrays;

import org.junit.Test;

public class SwerveModuleTest {
    private static final double DOUBLE_ERROR = 0.0001;

    @Test
    public void turnCounter() {
        try {
            PIDValue value = new PIDValue(1,1,1);
            SwerveModule module = new SwerveModule(true,0,0,false,0,0,0,value,0,false);
            module.setAngle(20);
            assertEquals(module.getAzimuthSetpoint(), 20, DOUBLE_ERROR);
            module.setAngle(350);
            assertEquals(module.getAzimuthSetpoint(), -10, DOUBLE_ERROR);
            module.setAngle(40);
            assertEquals(module.getAzimuthSetpoint(), 40, DOUBLE_ERROR);
            module.setAngle(120);
            module.setAngle(200);
            module.setAngle(285);
            module.setAngle(10);
            assertEquals(module.getAzimuthSetpoint(), 370, DOUBLE_ERROR);
            module.setAngle(0);
            assertEquals(module.getAzimuthSetpoint(), 360, DOUBLE_ERROR);
            module.setAngle(180);
            assertEquals(module.getAzimuthSetpoint(), 360, DOUBLE_ERROR);
        } catch(Exception exception) {
            System.out.println(exception.toString());
            System.out.println(Arrays.toString(exception.getStackTrace()));
            assertEquals(1,0);
        }
    }
}