package org.firstinspires.ftc.teamcode.control;

/** Bounded, measured coordinate search. No hardware access or changes to shared gains. */
public final class TurretAutoTuner {
    public enum State { IDLE, CENTERING, TESTING, COMPLETE, FAILED, CANCELLED }

    public static final class Gains {
        public final double p, d, s;

        public Gains(double p, double d, double s) {
            if (!finite(p, d, s) || p <= 0 || p > 0.2 || d < 0 || d > 0.02 || s < 0 || s > 0.3) {
                throw new IllegalArgumentException("Gains must satisfy 0 < P <= .2, 0 <= D <= .02, 0 <= S <= .3");
            }
            this.p = p;
            this.d = d;
            this.s = s;
        }
    }

    public static final int SEARCH_TRIALS = 13; // Baseline, then two passes over P/D/S.
    private final Gains baseline;
    private final double min, max, amplitude, stepSeconds, tolerance, maxGap;
    private State state = State.IDLE;
    private String message = "A: begin automatic sweep; B: cancel";
    private Gains candidate, best, pairAnchor;
    private double center, target, lastTime, runStart, phaseStart, stableSince = Double.NaN;
    private double integral, terminalPenalty, score, bestScore = Double.POSITIVE_INFINITY;
    private double baselineScore = Double.NaN, validationScore = Double.NaN;
    private double loopSum, maxLoop, overshoot, bestOvershoot, bestMeanError;
    private int loopCount, trial, leg, settledLegs;
    private boolean validating;

    public TurretAutoTuner(Gains baseline, double min, double max, double amplitude,
                           double stepSeconds, double tolerance, double maxGap) {
        if (baseline == null || !finite(min, max, amplitude, stepSeconds, tolerance, maxGap)
                || min >= max || amplitude < 5 || amplitude > 45 || stepSeconds < 0.6
                || stepSeconds > 2 || tolerance < 0.2 || tolerance > 3
                || maxGap < 0.04 || maxGap > 0.5) {
            throw new IllegalArgumentException("Invalid travel, sweep, tolerance or loop-gap settings");
        }
        this.baseline = baseline;
        this.min = min;
        this.max = max;
        this.amplitude = amplitude;
        this.stepSeconds = stepSeconds;
        this.tolerance = tolerance;
        this.maxGap = maxGap;
        best = candidate = pairAnchor = baseline;
    }

    public void start(double now, double position) {
        if (state != State.IDLE) return;
        if (!finite(now, position) || position - amplitude - 8 < min || position + amplitude + 8 > max) {
            fail("Start farther from the travel limits (sweep amplitude + 8 degrees clearance)");
            return;
        }
        center = target = position;
        runStart = lastTime = phaseStart = now;
        state = State.CENTERING;
        message = "Returning to sweep center";
    }

    /** Feed fresh angle/velocity once per TeleOp loop, before requesting the next command. */
    public void update(double now, double position, double velocity) {
        if (!isRunning()) return;
        double dt = now - lastTime;
        if (!finite(now, position, velocity) || dt <= 0 || dt > maxGap) {
            fail("Invalid feedback or excessive loop gap; inspect TeleOp loop timing");
            return;
        }
        lastTime = now;
        loopSum += dt;
        loopCount++;
        maxLoop = Math.max(maxLoop, dt);
        if (position < center - amplitude - 8 || position > center + amplitude + 8
                || position <= min || position >= max) {
            fail("Turret left the sweep envelope; check motor/encoder direction and calibration");
            return;
        }
        if (now - runStart > 150) {
            fail("Tuning exceeded 150 seconds");
            return;
        }
        if (state == State.CENTERING) {
            target = center;
            // A weak baseline may stick outside the final settling band. Returning near
            // center must still allow the search to reach stronger P/friction candidates.
            updateStable(now, Math.abs(center - position) <= Math.max(tolerance, 3) && Math.abs(velocity) <= 8);
            if (!Double.isNaN(stableSince) && now - stableSince >= 0.2) {
                integral = terminalPenalty = overshoot = 0;
                settledLegs = leg = 0;
                phaseStart = now;
                stableSince = Double.NaN;
                target = center + amplitude;
                state = State.TESTING;
                message = validating ? "Validating winner" : "Measuring step and moving-target response";
            } else if (now - phaseStart > 4) {
                fail("Could not settle at sweep center; check friction, feedback and power limit");
            }
            return;
        }

        double elapsed = now - phaseStart;
        double error = target - position; // Feedback belongs to the previous loop's command.
        integral += Math.abs(error) * dt;
        if (leg < 3) {
            double direction = leg == 1 ? -1 : 1;
            overshoot = Math.max(overshoot, Math.max(0, (position - target) * direction));
            updateStable(now, Math.abs(error) <= tolerance && Math.abs(velocity) <= 8);
            if (elapsed >= stepSeconds) {
                if (!Double.isNaN(stableSince) && now - stableSince >= 0.15) settledLegs++;
                terminalPenalty += Math.abs(error);
                leg++;
                phaseStart = now;
                stableSince = Double.NaN;
                target = leg == 1 ? center - amplitude : center;
            }
        } else {
            // One smooth revolution of the target tests the lag hidden by fixed-angle tests.
            double duration = 2 * stepSeconds;
            target = center + amplitude * Math.sin(2 * Math.PI * Math.min(elapsed / duration, 1));
            if (elapsed >= duration) finishTrial(now);
        }
    }

    private void updateStable(double now, boolean stable) {
        if (!stable) stableSince = Double.NaN;
        else if (Double.isNaN(stableSince)) stableSince = now;
    }

    private void finishTrial(double now) {
        double meanError = integral / (5 * stepSeconds);
        score = meanError + terminalPenalty / 3 + overshoot * 0.5 + (3 - settledLegs) * amplitude;
        if (trial == 0 && !validating) baselineScore = score;
        if (validating) {
            validationScore = score;
            if (settledLegs != 3 || score > bestScore * 1.25 + 0.1
                    || score > baselineScore * 1.1 + 0.1) {
                fail("Winner did not repeat reliably; no gains applied. Repeat with consistent load");
            } else {
                state = State.COMPLETE;
                target = center;
                message = "Validated. X: preview winner; Y: preview original; B: stop turret";
            }
            return;
        }
        // Penalized provisional winners let the search escape a baseline that cannot
        // settle. The independent validation still requires all three settled steps.
        if (score < bestScore) {
            bestScore = score;
            best = candidate;
            bestOvershoot = overshoot;
            bestMeanError = meanError;
        }
        trial++;
        if (trial == SEARCH_TRIALS) {
            validating = true;
            candidate = best;
        } else {
            if ((trial & 1) == 1) pairAnchor = best;
            double factor = trial <= 6 ? ((trial & 1) == 1 ? 0.65 : 1.5)
                    : ((trial & 1) == 1 ? 0.8 : 1.25);
            int axis = ((trial - 1) / 2) % 3;
            candidate = new Gains(
                    axis == 0 ? clip(pairAnchor.p * factor, 0.001, 0.2) : pairAnchor.p,
                    axis == 1 ? clip(Math.max(pairAnchor.d, 0.0005) * factor, 0, 0.02) : pairAnchor.d,
                    axis == 2 ? clip(Math.max(pairAnchor.s, 0.02) * factor, 0, 0.3) : pairAnchor.s);
        }
        state = State.CENTERING;
        phaseStart = now;
        stableSince = Double.NaN;
        target = center;
    }

    public void cancel() {
        if (state == State.COMPLETE) return;
        state = State.CANCELLED;
        message = "Cancelled; turret output is zero. A: restart";
    }

    public void fail(String reason) {
        state = State.FAILED;
        message = reason;
    }

    public boolean isRunning() { return state == State.CENTERING || state == State.TESTING; }
    public State getState() { return state; }
    public String getMessage() { return message; }
    public double getTarget() { return target; }
    public Gains getCommandGains() { return state == State.CENTERING ? best : candidate; }
    public Gains getBest() { return best; }
    public int getTrial() { return Math.min(trial + 1, SEARCH_TRIALS); }
    public boolean isValidating() { return validating; }
    public double getBaselineScore() { return baselineScore; }
    public double getBestScore() { return bestScore; }
    public double getValidationScore() { return validationScore; }
    public double getBestMeanError() { return bestMeanError; }
    public double getBestOvershoot() { return bestOvershoot; }
    public double getAverageLoopMs() { return loopCount == 0 ? 0 : 1000 * loopSum / loopCount; }
    public double getMaxLoopMs() { return maxLoop * 1000; }

    private static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
}
