package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class TurretPositionControllerTest {
    @Test
    public void frictionRampsNearTargetAndWorksInBothDirections() {
        assertEquals(0.04, calculate(2, 0, 0, 0, 0, 0.1, 12), 1e-12);
        assertEquals(-0.04, calculate(-2, 0, 0, 0, 0, 0.1, 12), 1e-12);
        assertEquals(0.1, calculate(20, 0, 0, 0, 0, 0.1, 12), 1e-12);
        assertEquals(0, calculate(0.1, 0, 0, 0.1, 0, 0.1, 12), 0);
    }

    @Test
    public void derivativeDampsMotionWithoutKickingOnTargetChanges() {
        assertEquals(0.15, calculate(10, 0, 5, 0.02, 0.01, 0, 12), 1e-12);
        assertEquals(0.25, calculate(10, 0, -5, 0.02, 0.01, 0, 12), 1e-12);
        assertEquals(0, calculate(90, 0, 0, 0, 0.01, 0, 12), 0);
    }

    @Test
    public void compensatesVoltageAndLimitsBothPowerDirections() {
        assertEquals(0.24, calculate(10, 0, 0, 0.02, 0, 0, 10), 1e-12);
        assertEquals(0.4, calculate(90, 0, 0, 1, 0, 0, 12), 0);
        assertEquals(-0.4, calculate(-90, 0, 0, 1, 0, 0, 12), 0);
    }

    @Test
    public void invalidFeedbackOrSettingsStopOutput() {
        assertEquals(0, calculate(10, Double.NaN, 0, 1, 0, 0, 12), 0);
        assertEquals(0, calculate(10, 0, Double.NaN, 1, 0, 0, 12), 0);
        assertEquals(0, calculate(10, 0, 0, 1, 0, 0, 0), 0);
        assertEquals(0, TurretPositionController.calculate(10, 0, 0, 1, 0, 0.1, 0, 0.15, 0.4, 12), 0);
        assertEquals(0, TurretPositionController.calculate(10, 0, 0, 1, 0, 0.1, 5, -1, 0.4, 12), 0);
    }

    private double calculate(double target, double position, double velocity,
                             double p, double d, double s, double voltage) {
        return TurretPositionController.calculate(target, position, velocity, p, d, s, 5, 0.15, 0.4, voltage);
    }
}
