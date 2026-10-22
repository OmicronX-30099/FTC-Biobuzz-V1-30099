package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class TurretAutoTunerTest {
    @Test
    public void measuredSearchImprovesSlowControllerAndValidatesWithoutChangingOriginal() {
        TurretAutoTuner.Gains original = new TurretAutoTuner.Gains(0.04, 0.004, 0.05);
        TurretAutoTuner tuner = tuner(original);
        Plant plant = new Plant();
        tuner.start(0, 0);
        simulate(tuner, plant, 0, 140);
        assertEquals(tuner.getMessage(), TurretAutoTuner.State.COMPLETE, tuner.getState());
        assertTrue(tuner.getBestScore() < tuner.getBaselineScore());
        assertTrue(tuner.getValidationScore() <= tuner.getBaselineScore() * 1.1 + 0.1);
        assertEquals(0.04, original.p, 0);
        assertEquals(0.004, original.d, 0);
        assertEquals(0.05, original.s, 0);
        assertTrue(tuner.getMaxLoopMs() < 11);
    }

    @Test
    public void refusesUnsafeStartingAngleAndStopsOnWrongDirectionTravel() {
        TurretAutoTuner tuner = tuner(new TurretAutoTuner.Gains(0.04, 0.00125, 0.05));
        tuner.start(0, 260);
        assertEquals(TurretAutoTuner.State.FAILED, tuner.getState());
        tuner = tuner(new TurretAutoTuner.Gains(0.04, 0.00125, 0.05));
        tuner.start(0, 0);
        tuner.update(0.01, -24, 0);
        assertEquals(TurretAutoTuner.State.FAILED, tuner.getState());
    }

    @Test
    public void rejectsStaleFeedbackNonFiniteFeedbackAndCancellation() {
        for (double[] sample : new double[][]{{0.2, 0, 0}, {0.01, Double.NaN, 0},
                {0.01, 0, Double.POSITIVE_INFINITY}, {-0.01, 0, 0}}) {
            TurretAutoTuner tuner = tuner(new TurretAutoTuner.Gains(0.04, 0.00125, 0.05));
            tuner.start(0, 0);
            tuner.update(sample[0], sample[1], sample[2]);
            assertEquals(TurretAutoTuner.State.FAILED, tuner.getState());
        }
        TurretAutoTuner tuner = tuner(new TurretAutoTuner.Gains(0.04, 0.00125, 0.05));
        tuner.start(0, 0);
        tuner.cancel();
        tuner.update(1, 100, 100);
        assertEquals(TurretAutoTuner.State.CANCELLED, tuner.getState());
        assertFalse(tuner.isRunning());
    }

    @Test
    public void stalledEncoderNeverProducesAnApprovedResult() {
        TurretAutoTuner tuner = tuner(new TurretAutoTuner.Gains(0.04, 0.00125, 0.05));
        tuner.start(0, 0);
        for (int i = 1; i <= 14000 && tuner.isRunning(); i++) tuner.update(i * 0.01, 0, 0);
        assertEquals(TurretAutoTuner.State.FAILED, tuner.getState());
    }

    @Test
    public void winnerMustRepeatUnderTheSameLoad() {
        TurretAutoTuner tuner = tuner(new TurretAutoTuner.Gains(0.04, 0.004, 0.05));
        Plant plant = new Plant();
        tuner.start(0, 0);
        double now = 0;
        while (tuner.isRunning() && !tuner.isValidating() && now < 140) {
            now += 0.01;
            plant.step(tuner);
            tuner.update(now, plant.position, plant.velocity);
        }
        assertTrue(tuner.isValidating());
        // Simulate an encoder that freezes when the validation steps begin.
        while (tuner.isRunning() && now < 150) {
            now += 0.01;
            if (tuner.getState() == TurretAutoTuner.State.CENTERING) plant.step(tuner);
            else plant.velocity = 0;
            tuner.update(now, plant.position, plant.velocity);
        }
        assertEquals(TurretAutoTuner.State.FAILED, tuner.getState());
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsUnboundedGains() {
        new TurretAutoTuner.Gains(1, 0, 0);
    }

    private static TurretAutoTuner tuner(TurretAutoTuner.Gains gains) {
        return new TurretAutoTuner(gains, -230, 270, 15, 1, 0.8, 0.15);
    }

    private static void simulate(TurretAutoTuner tuner, Plant plant, double start, double end) {
        for (double now = start + 0.01; now < end && tuner.isRunning(); now += 0.01) {
            plant.step(tuner);
            tuner.update(now, plant.position, plant.velocity);
        }
    }

    /** Inertia, viscous drag and Coulomb friction; deliberately independent of the search. */
    private static final class Plant {
        double position, velocity;

        void step(TurretAutoTuner tuner) {
            TurretAutoTuner.Gains gains = tuner.getCommandGains();
            double power = TurretPositionController.calculate(tuner.getTarget(), position, velocity,
                    gains.p, gains.d, gains.s, 5, 0.21, 0.4, 12);
            double friction = Math.abs(velocity) > 0.1 ? 0.04 * Math.signum(velocity)
                    : Math.min(Math.abs(power), 0.04) * Math.signum(power);
            velocity += ((power - friction) * 2500 - 8 * velocity) * 0.01;
            position += velocity * 0.01;
        }
    }
}
