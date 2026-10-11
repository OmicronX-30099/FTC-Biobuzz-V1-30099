package org.firstinspires.ftc.teamcode.control;

/**
 * Times velocity-drop recovery after the flywheel has first settled at its target.
 * A drop is a proxy for a shot; there is no ball sensor. Recovery includes the
 * configured dwell within the target band. Use a monotonic clock in seconds.
 */
public final class ShooterRecoveryTimer {
    public enum State { WAITING_FOR_TARGET, ARMED, RECOVERING, DISABLED, INVALID_INPUT }

    private State state = State.WAITING_FOR_TARGET;
    private double previousTarget = Double.NaN;
    private double previousTolerance = Double.NaN;
    private double previousDrop = Double.NaN;
    private double previousSettleMs = Double.NaN;
    private double stableSince = Double.NaN;
    private double recoverySince;
    private double minimumVelocity;
    private double lastRecoveryMs;
    private double lastDropTPS;
    private int completedRecoveries;

    public void reset() {
        state = State.WAITING_FOR_TARGET;
        previousTarget = Double.NaN;
        previousTolerance = Double.NaN;
        previousDrop = Double.NaN;
        previousSettleMs = Double.NaN;
        stableSince = Double.NaN;
        recoverySince = 0.0;
        minimumVelocity = 0.0;
        lastRecoveryMs = 0.0;
        lastDropTPS = 0.0;
        completedRecoveries = 0;
    }

    public void update(double nowSeconds, double target, double velocity,
                       double toleranceTPS, double dropTPS, double settleMs) {
        if (!Double.isFinite(nowSeconds) || !Double.isFinite(target)
                || !Double.isFinite(velocity) || !Double.isFinite(toleranceTPS)
                || !Double.isFinite(dropTPS) || !Double.isFinite(settleMs)
                || toleranceTPS < 0.0 || dropTPS <= toleranceTPS || settleMs < 0.0) {
            reset();
            state = State.INVALID_INPUT;
            return;
        }
        if (target < 1.0) {
            reset();
            state = State.DISABLED;
            return;
        }

        // Target or threshold changes begin a new measurement session, not a shot.
        if (target != previousTarget || toleranceTPS != previousTolerance
                || dropTPS != previousDrop || settleMs != previousSettleMs) {
            reset();
            previousTarget = target;
            previousTolerance = toleranceTPS;
            previousDrop = dropTPS;
            previousSettleMs = settleMs;
        }

        boolean inTargetBand = Math.abs(target - velocity) <= toleranceTPS;
        if (state == State.ARMED) {
            if (target - velocity >= dropTPS) {
                state = State.RECOVERING;
                recoverySince = nowSeconds;
                minimumVelocity = velocity;
                stableSince = Double.NaN;
            }
            return;
        }

        if (state == State.RECOVERING) {
            minimumVelocity = Math.min(minimumVelocity, velocity);
        }

        if (!inTargetBand) {
            stableSince = Double.NaN;
            return;
        }
        if (Double.isNaN(stableSince)) {
            stableSince = nowSeconds;
        }
        if ((nowSeconds - stableSince) * 1000.0 < settleMs) {
            return;
        }

        if (state == State.RECOVERING) {
            lastRecoveryMs = (nowSeconds - recoverySince) * 1000.0;
            lastDropTPS = target - minimumVelocity;
            completedRecoveries++;
        }
        state = State.ARMED;
        stableSince = Double.NaN;
    }

    public State getState() { return state; }
    public int getCompletedRecoveries() { return completedRecoveries; }
    public double getLastRecoveryMs() { return lastRecoveryMs; }
    public double getLastDropTPS() { return lastDropTPS; }

    public double getElapsedMs(double nowSeconds) {
        return state == State.RECOVERING ? (nowSeconds - recoverySince) * 1000.0 : 0.0;
    }
}
