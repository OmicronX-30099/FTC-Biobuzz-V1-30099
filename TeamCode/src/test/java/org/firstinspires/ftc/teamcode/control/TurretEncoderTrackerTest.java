package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

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

    @Test
    public void backupCorrectionRemovesDriftInBothDirectionsWithoutLosingMotorTurns() {
        for (int driftTicks : new int[]{-48, 48}) {
            TurretEncoderTracker tracker = tracker(false);
            tracker.seed(0, 0, 160);
            assertTrue(tracker.correctFromAbsolute(0, driftTicks, 15));
            assertEquals(160, tracker.positionDegrees(driftTicks), 1e-9);
            assertEquals(320, tracker.positionDegrees(driftTicks + 769), 1e-9);
            assertEquals(80, tracker.velocityDegreesPerSecond(384.5), 1e-9);
        }
    }

    @Test
    public void backupCorrectionHandlesAbsoluteWrapAndReversedRelativeEncoder() {
        TurretEncoderTracker tracker = tracker(true);
        tracker.seed(9, 0, 0); // -2 turret degrees before the absolute reading wraps
        assertTrue(tracker.correctFromAbsolute(355.5, 0, 15));
        assertEquals(1, tracker.positionDegrees(0), 1e-9);
        assertEquals(-159, tracker.positionDegrees(769), 1e-9);
    }

    @Test
    public void manualRevolutionSelectionPreservesFractionAndSurvivesBackupCorrection() {
        TurretEncoderTracker tracker = tracker(false);
        tracker.seed(337.5, 1000, 0); // +5 turret degrees
        tracker.shiftRevolutions(1);
        assertEquals(85, tracker.positionDegrees(1000), 1e-9);
        assertTrue(tracker.correctFromAbsolute(337.5, 1000, 15));
        assertEquals(85, tracker.positionDegrees(1000), 1e-9);
        assertEquals(245, tracker.positionDegrees(1769), 1e-9);
        assertEquals(80, tracker.velocityDegreesPerSecond(384.5), 1e-9);
        tracker.shiftRevolutions(-1);
        assertEquals(5, tracker.positionDegrees(1000), 1e-9);
        tracker.shiftRevolutions(-1);
        assertEquals(-75, tracker.positionDegrees(1000), 1e-9);
        assertTrue(tracker.correctFromAbsolute(337.5, 1000, 15));
        assertEquals(-75, tracker.positionDegrees(1000), 1e-9);
    }

    @Test
    public void backupCorrectionRejectsLargeOrInvalidDiscrepanciesWithoutChangingOffset() {
        TurretEncoderTracker tracker = tracker(false);
        tracker.seed(0, 0, 0);
        double before = tracker.positionDegrees(96); // nearly 20 degrees of discrepancy
        assertFalse(tracker.correctFromAbsolute(0, 96, 15));
        assertEquals(before, tracker.positionDegrees(96), 0);
        assertFalse(tracker.correctFromAbsolute(Double.NaN, 96, 15));
        assertFalse(tracker.correctFromAbsolute(361, 96, 15));
        assertFalse(tracker.correctFromAbsolute(0, 96, Double.NaN));
        assertFalse(tracker.correctFromAbsolute(0, 96, 40)); // half an absolute revolution is ambiguous
        assertEquals(before, tracker.positionDegrees(96), 0);
    }
}
