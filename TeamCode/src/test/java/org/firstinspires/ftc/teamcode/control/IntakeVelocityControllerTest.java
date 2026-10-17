package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class IntakeVelocityControllerTest {
    private static double power(double target, double measured) {
        return IntakeVelocityController.calculate(target, measured, 0.003, 0.09, 0.0004);
    }

    @Test
    public void suppliesFeedforwardInBothDirections() {
        assertEquals(0.49, power(1000.0, 1000.0), 1e-12);
        assertEquals(-0.49, power(-1000.0, -1000.0), 1e-12);
    }

    @Test
    public void recoversFromLoadInBothDirections() {
        assertEquals(0.79, power(1000.0, 900.0), 1e-12);
        assertEquals(-0.79, power(-1000.0, -900.0), 1e-12);
    }

    @Test
    public void limitsSpinupAndDirectionChangesToValidMotorPower() {
        assertEquals(1.0, power(1000.0, 0.0), 0.0);
        assertEquals(-1.0, power(-1000.0, 0.0), 0.0);
        assertEquals(-1.0, power(-1000.0, 1000.0), 0.0);
        assertEquals(1.0, power(1000.0, -1000.0), 0.0);
    }

    @Test
    public void appliesOppositePowerToReduceOverspeed() {
        assertEquals(-1.0, power(1000.0, 1500.0), 1e-12);
        assertEquals(1.0, power(-1000.0, -1500.0), 1e-12);
    }

    @Test
    public void stopAndSubTickTargetsDoNotDriveAgainstResidualMotion() {
        assertEquals(0.0, power(0.0, 1000.0), 0.0);
        assertEquals(0.0, power(0.0, -1000.0), 0.0);
        assertEquals(0.0, power(0.999, 0.0), 0.0);
        assertEquals(0.0, power(-0.999, 0.0), 0.0);
    }

    @Test
    public void rejectsNonFiniteInputsAndArithmeticOverflow() {
        double[] invalidValues = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double invalid : invalidValues) {
            assertEquals(0.0, power(invalid, 0.0), 0.0);
            assertEquals(0.0, power(1000.0, invalid), 0.0);
            assertEquals(0.0, IntakeVelocityController.calculate(1000, 0, invalid, 0.09, 0.0004), 0.0);
            assertEquals(0.0, IntakeVelocityController.calculate(1000, 0, 0.003, invalid, 0.0004), 0.0);
            assertEquals(0.0, IntakeVelocityController.calculate(1000, 0, 0.003, 0.09, invalid), 0.0);
        }
        assertEquals(0.0, power(Double.MAX_VALUE, -Double.MAX_VALUE), 0.0);
    }
}
