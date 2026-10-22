package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.Subsystem.Turret.ShotCalibration;
import org.junit.Test;

import static org.junit.Assert.*;

public class DistanceShotInterpolatorTest {
    @Test
    public void preservesEveryMeasuredShotIncludingTheCloseLob() {
        for (int i = 0; i < ShotCalibration.DISTANCE_INCHES.length; i++) {
            DistanceShotInterpolator.Shot shot = sample(ShotCalibration.DISTANCE_INCHES[i]);
            assertNotNull(shot);
            assertEquals(ShotCalibration.VELOCITY_TPS[i], shot.velocityTPS, 1e-9);
            assertEquals(ShotCalibration.HOOD_POSITIONS[i], shot.hoodPosition, 1e-12);
            assertFalse(shot.clamped);
        }
    }

    @Test
    public void interpolatesLobTransitionAndRetainsFlatHoodSegments() {
        DistanceShotInterpolator.Shot shot = sample((36.12014673281381 + 51.440559872536376) / 2);
        assertEquals(1460, shot.velocityTPS, 1e-9);
        assertEquals(0.3925, shot.hoodPosition, 1e-12);
        shot = sample((51.440559872536376 + 56.88717693821693) / 2);
        assertEquals(1510, shot.velocityTPS, 1e-9);
        assertEquals(0.275, shot.hoodPosition, 1e-12);
        shot = sample((56.88717693821693 + 70.41114968525936) / 2);
        assertEquals(1570, shot.velocityTPS, 1e-9);
        assertEquals(0.25, shot.hoodPosition, 1e-12);
    }

    @Test
    public void holdsMeasuredEndpointsInsteadOfExtrapolating() {
        DistanceShotInterpolator.Shot close = sample(0);
        assertEquals(1440, close.velocityTPS, 0);
        assertEquals(0.485, close.hoodPosition, 0);
        assertTrue(close.clamped);
        DistanceShotInterpolator.Shot far = sample(200);
        assertEquals(1600, far.velocityTPS, 0);
        assertEquals(0.25, far.hoodPosition, 0);
        assertTrue(far.clamped);
    }

    @Test
    public void rejectsInvalidDistanceAndBrokenLiveCalibrationTables() {
        assertNull(sample(Double.NaN));
        assertNull(sample(Double.POSITIVE_INFINITY));
        assertNull(sample(-1));
        assertNull(DistanceShotInterpolator.sample(10, null, new double[]{1, 2}, new double[]{0.2, 0.3}));
        assertNull(DistanceShotInterpolator.sample(10, new double[]{1, 2}, new double[]{1}, new double[]{0.2, 0.3}));
        assertNull(DistanceShotInterpolator.sample(10, new double[]{2, 1}, new double[]{1, 2}, new double[]{0.2, 0.3}));
        assertNull(DistanceShotInterpolator.sample(10, new double[]{1, 1}, new double[]{1, 2}, new double[]{0.2, 0.3}));
        assertNull(DistanceShotInterpolator.sample(10, new double[]{1, 2}, new double[]{1, -2}, new double[]{0.2, 0.3}));
        assertNull(DistanceShotInterpolator.sample(10, new double[]{1, 2}, new double[]{1, 2}, new double[]{0.2, 1.1}));
        assertNull(DistanceShotInterpolator.sample(10, new double[]{1, 2}, new double[]{1, Double.NaN}, new double[]{0.2, 0.3}));
    }

    private static DistanceShotInterpolator.Shot sample(double distance) {
        return DistanceShotInterpolator.sample(distance, ShotCalibration.DISTANCE_INCHES,
                ShotCalibration.VELOCITY_TPS, ShotCalibration.HOOD_POSITIONS);
    }

    @Test
    public void reusableOutputHandlesRangeChangesAndInvalidCalibrationWithoutStaleCommands() {
        DistanceShotInterpolator.Shot output = new DistanceShotInterpolator.Shot();
        double[] distances = {10, 20};
        double[] speeds = {1000, 1200};
        double[] hoods = {0.3, 0.4};
        assertTrue(DistanceShotInterpolator.sampleInto(30, distances, speeds, hoods, output));
        assertEquals(1200, output.velocityTPS, 0);
        assertTrue(output.clamped);
        assertTrue(DistanceShotInterpolator.sampleInto(15, distances, speeds, hoods, output));
        assertEquals(1100, output.velocityTPS, 0);
        assertEquals(0.35, output.hoodPosition, 1e-12);
        assertFalse(output.clamped);
        speeds[1] = Double.NaN;
        assertFalse(DistanceShotInterpolator.sampleInto(15, distances, speeds, hoods, output));
        assertTrue(Double.isNaN(output.velocityTPS));
        assertTrue(Double.isNaN(output.hoodPosition));
        speeds[1] = 1200;
        assertTrue(DistanceShotInterpolator.sampleInto(0, distances, speeds, hoods, output));
        assertEquals(1000, output.velocityTPS, 0);
        assertTrue(output.clamped);
    }
}
