package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;

public class DistanceShotModelTest {
    @Test
    public void evaluatesRequestedPolynomialWithoutTheOldVelocityOffset() {
        assertEquals(2295.8006, DistanceShotModel.velocityTPS(0), 1e-9);
        assertEquals(1507.41249858, DistanceShotModel.velocityTPS(23), 1e-9);
        assertEquals(1477.27902528, DistanceShotModel.velocityTPS(38), 1e-9);
        assertEquals(1679.89819968, DistanceShotModel.velocityTPS(68), 1e-9);
    }

    @Test
    public void interpolatesHoodWhileVelocityUsesActualDistance() {
        double[] distances = {23, 38, 68};
        double[] hoods = {0.48, 0.30, 0.20};
        DistanceShotInterpolator.Shot shot = DistanceShotModel.sample(30.5, distances, hoods);
        assertNotNull(shot);
        assertEquals(0.39, shot.hoodPosition, 1e-12);
        assertEquals(DistanceShotModel.velocityTPS(30.5), shot.velocityTPS, 1e-9);
        assertFalse(shot.clamped);
        shot = DistanceShotModel.sample(80, distances, hoods);
        assertNotNull(shot);
        assertEquals(0.20, shot.hoodPosition, 0);
        assertEquals(1582.60428, shot.velocityTPS, 1e-9);
        assertTrue(shot.clamped);
    }

    @Test
    public void invalidDistanceOrNegativePolynomialOutputInvalidatesShot() {
        double[] distances = {23, 68};
        double[] hoods = {0.48, 0.20};
        for (double distance : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -1, 120}) {
            assertNull(DistanceShotModel.sample(distance, distances, hoods));
        }
        DistanceShotInterpolator.Shot output = DistanceShotModel.sample(38, distances, hoods);
        assertFalse(DistanceShotModel.sampleInto(Double.NaN, distances, hoods, output));
        assertTrue(Double.isNaN(output.velocityTPS));
        assertTrue(Double.isNaN(output.hoodPosition));
    }
}
