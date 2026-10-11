package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ShooterRecoveryTimerTest {
    private final ShooterRecoveryTimer timer = new ShooterRecoveryTimer();

    private void sample(double seconds, double velocity) {
        timer.update(seconds, 1500.0, velocity, 50.0, 100.0, 100.0);
    }

    private void arm() {
        sample(0.0, 1500.0);
        sample(0.101, 1500.0);
        assertEquals(ShooterRecoveryTimer.State.ARMED, timer.getState());
    }

    @Test
    public void initialSpinupIsNotCountedAsRecovery() {
        sample(0.0, 0.0);
        sample(0.2, 1300.0);
        sample(0.3, 1500.0);
        sample(0.35, 1500.0);
        assertEquals(ShooterRecoveryTimer.State.WAITING_FOR_TARGET, timer.getState());
        sample(0.401, 1500.0);
        assertEquals(ShooterRecoveryTimer.State.ARMED, timer.getState());
        assertEquals(0, timer.getCompletedRecoveries());
    }

    @Test
    public void measuresRecoveryIncludingStableDwellAndMaximumDroop() {
        arm();
        sample(0.2, 1300.0);
        sample(0.25, 1100.0);
        assertEquals(ShooterRecoveryTimer.State.RECOVERING, timer.getState());
        assertEquals(100.0, timer.getElapsedMs(0.3), 1e-9);
        sample(0.5, 1470.0);
        sample(0.55, 1500.0);
        assertEquals(0, timer.getCompletedRecoveries());
        sample(0.601, 1500.0);
        assertEquals(ShooterRecoveryTimer.State.ARMED, timer.getState());
        assertEquals(401.0, timer.getLastRecoveryMs(), 1e-9);
        assertEquals(400.0, timer.getLastDropTPS(), 1e-9);
        assertEquals(1, timer.getCompletedRecoveries());
        assertEquals(0.0, timer.getElapsedMs(0.7), 0.0);
    }

    @Test
    public void noiseAndOverspeedDoNotTriggerRecovery() {
        arm();
        sample(0.2, 1450.0);
        sample(0.3, 1401.0);
        sample(0.4, 1700.0);
        assertEquals(ShooterRecoveryTimer.State.ARMED, timer.getState());
        assertEquals(0, timer.getCompletedRecoveries());
        sample(0.5, 1400.0);
        assertEquals(ShooterRecoveryTimer.State.RECOVERING, timer.getState());
    }

    @Test
    public void transientTargetCrossingDoesNotFinishRecovery() {
        arm();
        sample(0.2, 1200.0);
        sample(0.3, 1500.0);
        sample(0.35, 1560.0);
        sample(0.4, 1500.0);
        sample(0.45, 1500.0);
        assertEquals(ShooterRecoveryTimer.State.RECOVERING, timer.getState());
        sample(0.501, 1500.0);
        assertEquals(301.0, timer.getLastRecoveryMs(), 1e-9);
    }

    @Test
    public void canMeasureSuccessiveRecoveries() {
        arm();
        sample(0.2, 1200.0);
        sample(0.3, 1500.0);
        sample(0.401, 1500.0);
        sample(0.5, 1300.0);
        sample(0.6, 1500.0);
        sample(0.701, 1500.0);
        assertEquals(2, timer.getCompletedRecoveries());
        assertEquals(201.0, timer.getLastRecoveryMs(), 1e-9);
        assertEquals(200.0, timer.getLastDropTPS(), 1e-9);
    }

    @Test
    public void targetChangesAbortRecoveryAndRequireRearming() {
        arm();
        sample(0.2, 1200.0);
        timer.update(0.3, 1800.0, 1500.0, 50.0, 100.0, 100.0);
        assertEquals(ShooterRecoveryTimer.State.WAITING_FOR_TARGET, timer.getState());
        assertEquals(0, timer.getCompletedRecoveries());
        assertEquals(0.0, timer.getElapsedMs(0.4), 0.0);
    }

    @Test
    public void zeroTargetAndInvalidThresholdsDisableMeasurement() {
        arm();
        timer.update(0.2, 0.0, 1500.0, 50.0, 100.0, 100.0);
        assertEquals(ShooterRecoveryTimer.State.DISABLED, timer.getState());
        timer.update(0.3, 1500.0, 1500.0, 50.0, 25.0, 100.0);
        assertEquals(ShooterRecoveryTimer.State.INVALID_INPUT, timer.getState());
        timer.update(0.4, 1500.0, Double.NaN, 50.0, 100.0, 100.0);
        assertEquals(ShooterRecoveryTimer.State.INVALID_INPUT, timer.getState());
    }

    @Test
    public void thresholdChangesBeginANewMeasurementSession() {
        arm();
        sample(0.2, 1200.0);
        timer.update(0.3, 1500.0, 1500.0, 25.0, 100.0, 100.0);
        assertEquals(ShooterRecoveryTimer.State.WAITING_FOR_TARGET, timer.getState());
        timer.update(0.401, 1500.0, 1500.0, 25.0, 100.0, 100.0);
        assertEquals(ShooterRecoveryTimer.State.ARMED, timer.getState());
        assertEquals(0, timer.getCompletedRecoveries());
    }
}
