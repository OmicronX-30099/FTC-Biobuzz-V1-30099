package org.firstinspires.ftc.teamcode.control;

/**
 * Proportional velocity feedback plus static and velocity feedforward.
 * Ported from FTC-Decode-v3's Flywheel.calculatePow() in ShooterUtil.kt.
 * Velocities are encoder ticks per second; output is motor power from 0 to 1.
 * This forward-only controller lets the flywheel coast when above its target.
 */
public final class ShooterVelocityController {
    public static final double DEFAULT_KP = 0.003;
    public static final double DEFAULT_KS = 0.04;
    public static final double DEFAULT_KV = 0.00038;

    private ShooterVelocityController() { }

    public static double calculate(double targetVelocity, double currentVelocity,
                                   double kP, double kS, double kV) {
        if (!Double.isFinite(targetVelocity) || !Double.isFinite(currentVelocity)
                || !Double.isFinite(kP) || !Double.isFinite(kS) || !Double.isFinite(kV)
                || Math.abs(targetVelocity) < 1.0) {
            return 0.0;
        }

        double error = targetVelocity - currentVelocity;
        double feedforward = kV * targetVelocity + kS * Math.signum(targetVelocity);
        return Math.max(0.0, Math.min(1.0, feedforward + kP * error));
    }
}
