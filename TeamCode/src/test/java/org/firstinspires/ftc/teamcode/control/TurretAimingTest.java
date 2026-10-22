package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;

public class TurretAimingTest {
    @Test
    public void subtractsRobotHeadingUsingPedroFieldAxes() {
        assertEquals(0, aim(0, 0, Math.PI / 2, 0, 10, 0), 1e-9);
        assertEquals(-90, aim(0, 0, Math.PI / 2, 10, 0, 0), 1e-9);
        assertEquals(45, aim(0, 0, 0, 10, 10, 0), 1e-9);
    }

    @Test
    public void aimsAtConfiguredTargetFromRequestedStartingPose() {
        double target = TurretAiming.targetDegrees(8.2, 7.3, Math.PI / 2, 57.9, 51.2,
                0, -230, 270, true);
        assertEquals(48.5458457364, target, 1e-6);
    }

    @Test
    public void convertsCounterclockwiseFieldBearingsToClockwiseEncoderAngles() {
        assertEquals(90, TurretAiming.targetDegrees(0, 0, Math.PI / 2, 10, 0,
                0, -230, 270, true), 1e-9);
        assertEquals(-45, TurretAiming.targetDegrees(0, 0, 0, 10, 10,
                0, -230, 270, true), 1e-9);
        assertEquals(260, TurretAiming.targetDegrees(0, 0, Math.toRadians(-100), 10, 0,
                250, -230, 270, true), 1e-9);
    }

    @Test
    public void usesClosestEquivalentInsideWireLimits() {
        assertEquals(260, aim(0, 0, Math.toRadians(100), 10, 0, 250), 1e-9);
        assertEquals(-220, aim(0, 0, Math.toRadians(-140), 10, 0, -210), 1e-9);
        // +275 would exceed the upper limit, so the turret must return to -85.
        assertEquals(-85, aim(0, 0, Math.toRadians(85), 10, 0, 260), 1e-9);
    }

    @Test
    public void appliesPhysicalRightTrimBeforeSelectingTheLegalRevolution() {
        assertEquals(2, TurretAiming.targetDegrees(0, 0, 0, 10, 0,
                0, -230, 270, true, 2), 1e-9);
        assertEquals(-2, TurretAiming.targetDegrees(0, 0, 0, 10, 0,
                0, -230, 270, false, 2), 1e-9);
        assertEquals(-88, TurretAiming.targetDegrees(0, 0, Math.toRadians(268), 10, 0,
                260, -230, 270, true, 4), 1e-9);
        assertTrue(Double.isNaN(TurretAiming.targetDegrees(0, 0, 0, 10, 0,
                0, -230, 270, true, Double.NaN)));
    }

    @Test
    public void holdsAngleAtCoincidentTargetAndRejectsInvalidPose() {
        assertEquals(200, aim(1, 2, 0, 1, 2, 200), 0);
        assertTrue(Double.isNaN(aim(Double.NaN, 0, 0, 10, 0, 0)));
    }

    private double aim(double x, double y, double heading, double goalX, double goalY, double current) {
        return TurretAiming.targetDegrees(x, y, heading, goalX, goalY, current, -230, 270, false);
    }
}
