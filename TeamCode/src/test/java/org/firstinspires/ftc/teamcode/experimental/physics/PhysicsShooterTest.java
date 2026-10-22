package org.firstinspires.ftc.teamcode.experimental.physics;

import org.junit.Test;
import static org.junit.Assert.*;

public class PhysicsShooterTest {
    @Test
    public void lookupRemainsBoundedDescendingAndRespectsVertexGeometryAcrossRange() {
        PhysicsShot shot = new PhysicsShot();
        for (int i = 0; i <= 320; i++) {
            double distance = 35 + i * 0.125;
            assertSame(shot, PhysicsShooter.calculateShot(distance, shot));
            assertTrue(shot.getValid());
            assertTrue(shot.getHoodAngleDegrees() >= 15 && shot.getHoodAngleDegrees() <= 36);
            assertEquals(90-shot.getHoodAngleDegrees(), shot.getLaunchAngleDegrees(), 1e-9);
            assertTrue(shot.getShooterVelocityTPS() > 0);
            assertTrue(shot.getEntryAngleDegrees() > 0 && shot.getEntryAngleDegrees() < 90);
            assertTrue(shot.getVertexDistanceInches() > 0 && shot.getVertexDistanceInches() < distance);
            assertTrue(shot.getFlightTimeSeconds() > 0);
            assertEquals(Math.abs(shot.getEntryAngleDegrees()-30), shot.getEntryAngleErrorDegrees(), 1e-9);
            if (shot.getFeasible()) {
                assertTrue(shot.getHeightConstraintSatisfied());
                assertTrue(shot.getVertexHeightCm() >= 136);
            }
        }
    }

    @Test
    public void shortRangeReturnsHonestFallbackAndInvalidDistancesClearStaleShot() {
        PhysicsShot shot = new PhysicsShot();
        PhysicsShooter.calculateShot(35, shot);
        assertTrue(shot.getValid());
        assertFalse(shot.getFeasible());
        assertFalse(shot.getHeightConstraintSatisfied());
        assertEquals(15, shot.getHoodAngleDegrees(), 0);
        assertTrue(shot.getVertexHeightCm() < 135);
        PhysicsShooter.calculateShot(75, shot);
        assertTrue(shot.getFeasible());
        for (double invalid : new double[] {34.99, 75.01, Double.NaN, Double.POSITIVE_INFINITY, -1}) {
            PhysicsShooter.calculateShot(invalid, shot);
            assertFalse(shot.getValid());
            assertFalse(shot.getFeasible());
            assertTrue(Double.isNaN(shot.getShooterVelocityTPS()));
            assertTrue(Double.isNaN(shot.getHoodAngleDegrees()));
        }
    }

    @Test
    public void hoodMappingHonorsMechanicalEndpointsAndRejectsInvalidAngles() {
        assertEquals(.485, PhysicsShooter.hoodServoPosition(15), 1e-12);
        double travel = 355 * .9 * 30 / 148;
        assertEquals(.485 - 21 / travel, PhysicsShooter.hoodServoPosition(36), 1e-12);
        assertTrue(PhysicsShooter.hoodServoPosition(36) > .16);
        double servo = PhysicsShooter.hoodServoPosition(27);
        assertEquals(27, 15 + (.485-servo)*travel, 1e-12);
        assertEquals(600, PhysicsShooter.PWM_MIN_US, 0);
        assertEquals(2400, PhysicsShooter.PWM_MAX_US, 0);
        assertTrue(Double.isNaN(PhysicsShooter.hoodServoPosition(14.99)));
        assertTrue(Double.isNaN(PhysicsShooter.hoodServoPosition(36.01)));
        assertTrue(Double.isNaN(PhysicsShooter.hoodServoPosition(Double.NaN)));
    }

    @Test
    public void analyticSpeedHitsTargetAtPreferredEntryAngle() {
        assertEquals(104, PhysicsShooter.TARGET_HEIGHT_CM, 0);
        assertEquals(30, PhysicsShooter.PREFERRED_ENTRY_DEGREES, 0);
        double targetHeightM = PhysicsShooter.TARGET_HEIGHT_CM / 100.0;
        for (double distanceIn : new double[] {35, 50, 60, 75}) {
            double d = distanceIn*.0254;
            double theta = Math.atan(2*targetHeightM/d+Math.tan(Math.toRadians(30)));
            double speed = IdealProjectile.requiredSpeed(d, targetHeightM, Math.toDegrees(theta));
            double vx = speed*Math.cos(theta);
            double time = d/vx;
            double vz = speed*Math.sin(theta) - IdealProjectile.GRAVITY*time;
            double height = speed*Math.sin(theta)*time - .5*IdealProjectile.GRAVITY*time*time;
            assertEquals(targetHeightM, height, 1e-9);
            assertEquals(30, Math.toDegrees(Math.atan2(-vz, vx)), 1e-9);
        }
        assertTrue(Double.isNaN(IdealProjectile.requiredSpeed(0, targetHeightM, 75)));
        assertTrue(Double.isNaN(IdealProjectile.requiredSpeed(1, targetHeightM, 30)));
    }
}
