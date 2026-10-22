package org.firstinspires.ftc.teamcode.control;

/** Software PD position control with friction compensation ramped down near the target. */
public final class TurretPositionController {
    private TurretPositionController() { }

    /**
     * Angles are turret degrees, velocity is turret degrees/second, and output is motor power.
     * D acts on measured velocity to avoid a derivative kick when changing the target.
     */
    public static double calculate(double targetDegrees, double currentDegrees,
                                   double velocityDegreesPerSecond, double kP, double kD,
                                   double kS, double frictionRampDegrees, double deadbandDegrees,
                                   double maxPower, double voltage) {
        if (!Double.isFinite(targetDegrees) || !Double.isFinite(currentDegrees)
                || !Double.isFinite(velocityDegreesPerSecond) || !Double.isFinite(kP)
                || !Double.isFinite(kD) || !Double.isFinite(kS)
                || !Double.isFinite(frictionRampDegrees) || frictionRampDegrees <= 0
                || !Double.isFinite(deadbandDegrees) || deadbandDegrees < 0
                || !Double.isFinite(maxPower) || maxPower < 0
                || !Double.isFinite(voltage) || voltage <= 0) {
            return 0;
        }

        double error = targetDegrees - currentDegrees;
        if (Math.abs(error) <= deadbandDegrees) return 0;

        double friction = kS * Math.signum(error)
                * Math.min(Math.abs(error) / frictionRampDegrees, 1.0);
        double output = (kP * error - kD * velocityDegreesPerSecond + friction)
                * 12.0 / Math.max(voltage, 1.0);
        double limit = Math.min(maxPower, 1.0);
        return Math.max(-limit, Math.min(limit, output));
    }
}
