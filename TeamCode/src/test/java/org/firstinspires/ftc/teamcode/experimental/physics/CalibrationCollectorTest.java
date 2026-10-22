package org.firstinspires.ftc.teamcode.experimental.physics;

import org.junit.Test;
import java.util.Locale;
import static org.junit.Assert.*;

public class CalibrationCollectorTest {
    @Test
    public void manualControlsChangeOncePerPressAndResetToKnownSuccessfulBaseline() {
        CalibrationSettings settings = new CalibrationSettings();
        assertEquals(1480, settings.getTargetTPS(), 0);
        assertEquals(.3, settings.getHoodPosition(), 0);
        settings.update(false, true, true, false);
        assertEquals(1500, settings.getTargetTPS(), 0);
        assertEquals(.305, settings.getHoodPosition(), 1e-12);
        settings.update(false, true, true, false);
        assertEquals(1500, settings.getTargetTPS(), 0);
        settings.update(false, false, false, false);
        settings.update(true, false, false, true);
        assertEquals(1480, settings.getTargetTPS(), 0);
        assertEquals(.30, settings.getHoodPosition(), 1e-12);
        settings.reset(true, false, false, true);
        settings.update(true, false, false, true);
        assertEquals(1480, settings.getTargetTPS(), 0);
        assertEquals(.30, settings.getHoodPosition(), 1e-12);
    }

    @Test
    public void manualHoodLimitsRemainInsideMechanicalRangeAndSpeedCannotReverse() {
        CalibrationSettings settings = new CalibrationSettings();
        for (int i = 0; i < 200; i++) {
            settings.update(false, false, false, false);
            settings.update(true, false, false, true);
        }
        assertEquals(0, settings.getTargetTPS(), 0);
        assertEquals(36, PhysicsShooter.hoodDegreesFromServoPosition(settings.getHoodPosition()), 1e-9);
        for (int i = 0; i < 200; i++) {
            settings.update(false, false, false, false);
            settings.update(false, true, false, false);
        }
        assertEquals(15, PhysicsShooter.hoodDegreesFromServoPosition(settings.getHoodPosition()), 1e-9);
        assertTrue(Double.isNaN(PhysicsShooter.hoodDegreesFromServoPosition(Double.NaN)));
        assertTrue(Double.isNaN(PhysicsShooter.hoodDegreesFromServoPosition(.1)));
    }

    @Test
    public void csvPreservesPreFeedVelocityPoseSettingsAndOperatorResult() {
        CalibrationLog log = new CalibrationLog();
        assertFalse(log.rateLast(true));
        assertTrue(log.capture(1.25, 59.35, 7.68, .5, 1, 56, 59, 1480, 1479.5, .30, true));
        assertTrue(log.rateLast(true));
        assertFalse(log.rateLast(false));
        // A later attempt at different settings must not change the first sample's pre-feed reading.
        assertTrue(log.capture(4.5, 59.35, 7.68, .5, 1, 56, 59, 1500, 1490, .305, false));
        assertTrue(log.rateLast(false));
        assertEquals(2, log.getCount());
        assertEquals(1, log.getScores());
        assertEquals(1, log.getMisses());
        assertEquals(0, log.getUnrated());
        Locale saved = Locale.getDefault();
        try {
            Locale.setDefault(Locale.GERMANY);
            String[] lines = log.toCsv().trim().split("\n");
            String[] first = lines[1].split(",");
            assertEquals(lines[0].split(",").length, first.length);
            assertEquals("1480.000000", first[9]);
            assertEquals("1479.500000", first[10]);
            assertEquals("0.300000", first[11]);
            assertEquals("26.981250", first[12]);
            assertEquals("true", first[13]);
            assertEquals("SCORE", first[14]);
            assertEquals("104.0", first[15]);
            assertEquals("600.0", first[16]);
            assertEquals("2400.0", first[17]);
            assertEquals("MISS", lines[2].split(",")[14]);
        } finally {
            Locale.setDefault(saved);
        }
    }

    @Test
    public void invalidAttemptsAreRejectedAndUnratedAttemptsAreRetained() {
        CalibrationLog log = new CalibrationLog();
        assertFalse(log.capture(0, Double.NaN, 7, 0, 1, 56, 59, 1480, 1480, .30, true));
        assertFalse(log.capture(0, 56, 59, 0, 1, 56, 59, 1480, 1480, .30, true));
        assertFalse(log.capture(0, 10, 7, 0, 1, 56, 59, 0, 0, .30, false));
        assertFalse(log.capture(0, 10, 7, 0, 1, 56, 59, 1480, Double.NaN, .30, true));
        assertEquals(0, log.getCount());
        assertTrue(log.capture(1, 10, 7, 0, 1, 56, 59, 1480, 1480, .30, true));
        assertTrue(log.capture(2, 10, 7, 0, 1, 56, 59, 1480, 1480, .30, true));
        assertTrue(log.rateLast(true));
        assertEquals(1, log.getUnrated());
        assertTrue(log.toCsv().contains("UNRATED"));
    }
}
