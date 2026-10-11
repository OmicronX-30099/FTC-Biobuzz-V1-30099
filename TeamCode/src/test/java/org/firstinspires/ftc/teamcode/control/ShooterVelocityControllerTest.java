package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ShooterVelocityControllerTest {
    private static double power(double target, double measured) {
        return ShooterVelocityController.calculate(target, measured, 0.003, 0.09, 0.0004);
    }

    @Test
    public void matchesLastSeasonsFeedforwardAtTarget() {
        assertEquals(0.49, power(1000.0, 1000.0), 1e-12);
        assertEquals(0.69, power(1500.0, 1500.0), 1e-12);
        assertEquals(0.89, power(2000.0, 2000.0), 1e-12);
    }

    @Test
    public void addsFeedbackForVelocityDroop() {
        assertEquals(0.79, power(1000.0, 900.0), 1e-12);
    }

    @Test
    public void clipsSpinupToFullPowerAndOverspeedToCoasting() {
        assertEquals(1.0, power(1000.0, 0.0), 0.0);
        assertEquals(0.0, power(1000.0, 1500.0), 0.0);
    }

    @Test
    public void zeroAndSubTickTargetsStopEvenWithVelocityError() {
        assertEquals(0.0, power(0.0, -1000.0), 0.0);
        assertEquals(0.0, power(0.999, 0.0), 0.0);
        assertEquals(0.0, power(-0.999, 0.0), 0.0);
    }

    @Test
    public void updatedGainsTakeEffectOnTheNextCalculation() {
        assertEquals(0.59, ShooterVelocityController.calculate(
                1000.0, 900.0, 0.001, 0.09, 0.0004), 1e-12);
        assertEquals(0.79, power(1000.0, 900.0), 1e-12);
    }

    @Test
    public void invalidInputsDoNotCommandMotorPower() {
        assertEquals(0.0, power(Double.NaN, 0.0), 0.0);
        assertEquals(0.0, power(1000.0, Double.POSITIVE_INFINITY), 0.0);
        assertEquals(0.0, ShooterVelocityController.calculate(
                1000.0, 0.0, Double.NaN, 0.09, 0.0004), 0.0);
    }
}
