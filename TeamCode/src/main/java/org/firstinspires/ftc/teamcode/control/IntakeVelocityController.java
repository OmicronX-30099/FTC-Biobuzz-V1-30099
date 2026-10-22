package org.firstinspires.ftc.teamcode.control;

/**
 * Proportional velocity feedback plus static and velocity feedforward.
 * Velocities are signed encoder ticks per second; output is motor power from -1 to 1.
 * Positive targets collect and negative targets eject when motor direction is configured
 * accordingly. Feedback can command opposite power to reduce overspeed.
 */
public final class IntakeVelocityController {
    // Tuned intake gains, shared by Rollers and IntakeTest.
    public static final double DEFAULT_KP = 0.0006;
    public static final double DEFAULT_KS = 0.03;
    public static final double DEFAULT_KV = 0.00035;

    private IntakeVelocityController() { }

    /** Returns zero for invalid inputs or targets below one tick/s, including stop. */
    public static double calculate(double targetVelocity, double currentVelocity,
                                   double kP, double kS, double kV) {
        if (!Double.isFinite(targetVelocity) || !Double.isFinite(currentVelocity)
                || !Double.isFinite(kP) || !Double.isFinite(kS) || !Double.isFinite(kV)
                || Math.abs(targetVelocity) < 1.0) {
            return 0.0;
        }

        double error = targetVelocity - currentVelocity;
        double feedforward = kV * targetVelocity + kS * Math.signum(targetVelocity);
        double power = feedforward + kP * error;
        return Double.isFinite(power) ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
    }
}
