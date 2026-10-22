package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.assertEquals;

public class TurretEncoderTrackerTest {
    @Test
    public void selectsAbsoluteRevolutionUsingStartingEstimate() {
        TurretEncoderTracker tracker = tracker(false);
        assertEquals(0, tracker.estimateDegrees(0, 0), 0);
        assertEquals(160, tracker.estimateDegrees(0, 160), 0);
        assertEquals(230, tracker.estimateDegrees(45, 230), 0);
        assertEquals(-230, tracker.estimateDegrees(315, -230), 0);
    }

    @Test
    public void tracksAcrossMultipleMotorRevolutionsAndRestartsWithoutResettingHardware() {
        TurretEncoderTracker tracker = tracker(false);
        tracker.seed(0, 1000, 0);
        assertEquals(160, tracker.positionDegrees(1769), 1e-9);
        assertEquals(-160, tracker.positionDegrees(231), 1e-9);
        tracker.seed(0, 1769, 160);
        assertEquals(160, tracker.positionDegrees(1769), 1e-9);
        assertEquals(320, tracker.positionDegrees(2538), 1e-9);
    }

    @Test
    public void convertsVelocityAndHonorsIndependentRelativeDirection() {
        assertEquals(80, tracker(false).velocityDegreesPerSecond(384.5), 1e-9);
        assertEquals(-80, tracker(true).velocityDegreesPerSecond(384.5), 1e-9);
        TurretEncoderTracker reversed = tracker(true);
        reversed.seed(0, 0, 0);
        assertEquals(-160, reversed.positionDegrees(769), 1e-9);
    }

    @Test
    public void absoluteWrapNearZeroSeedsASmallAngle() {
        assertEquals(1.0 / 4.5, tracker(false).estimateDegrees(359, 0), 1e-9);
        assertEquals(-1.0 / 4.5, tracker(false).estimateDegrees(1, 0), 1e-9);
    }

    private TurretEncoderTracker tracker(boolean relativeReversed) {
        return new TurretEncoderTracker(384.5, 4.5, 0, true, relativeReversed);
    }
}
