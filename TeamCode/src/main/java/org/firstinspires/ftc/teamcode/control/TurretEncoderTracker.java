package org.firstinspires.ftc.teamcode.control;

/** Seeds an unwrapped turret angle from a motor-side absolute encoder, then uses relative ticks. */
public final class TurretEncoderTracker {
    private final double gearRatio;
    private final double degreesPerTick;
    private final double absoluteZero;
    private final double absoluteSign;
    private int initialTicks;
    private double initialDegrees;

    public TurretEncoderTracker(double ticksPerMotorRev, double motorRevsPerTurretRev,
                                double absoluteZeroDegrees, boolean absoluteReversed,
                                boolean relativeReversed) {
        if (!Double.isFinite(ticksPerMotorRev) || ticksPerMotorRev <= 0
                || !Double.isFinite(motorRevsPerTurretRev) || motorRevsPerTurretRev <= 0
                || !Double.isFinite(absoluteZeroDegrees)) {
            throw new IllegalArgumentException("Turret encoder calibration requires positive ratios and a finite zero.");
        }
        gearRatio = motorRevsPerTurretRev;
        degreesPerTick = (relativeReversed ? -1.0 : 1.0) * 360.0 / (ticksPerMotorRev * gearRatio);
        absoluteZero = absoluteZeroDegrees;
        absoluteSign = absoluteReversed ? -1.0 : 1.0;
    }

    public double estimateDegrees(double absoluteMotorDegrees, double approximateTurretDegrees) {
        double motorDegrees = absoluteSign * (absoluteMotorDegrees - absoluteZero);
        motorDegrees = ((motorDegrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        double baseDegrees = motorDegrees / gearRatio;
        double repeatDegrees = 360.0 / gearRatio;
        return baseDegrees + repeatDegrees * Math.round((approximateTurretDegrees - baseDegrees) / repeatDegrees);
    }

    public void seed(double absoluteMotorDegrees, int relativeTicks, double approximateTurretDegrees) {
        initialDegrees = estimateDegrees(absoluteMotorDegrees, approximateTurretDegrees);
        initialTicks = relativeTicks;
    }

    public double positionDegrees(int relativeTicks) {
        return initialDegrees + (relativeTicks - initialTicks) * degreesPerTick;
    }

    /** Select another absolute-encoder revolution without resetting ticks or fractional angle. */
    public void shiftRevolutions(int revolutions) {
        initialDegrees += revolutions * (360.0 / gearRatio);
    }

    /** Correct drift using the nearest absolute revolution, preserving relative velocity and ticks. */
    public boolean correctFromAbsolute(double absoluteMotorDegrees, int relativeTicks,
                                       double maxCorrectionDegrees) {
        if (!Double.isFinite(absoluteMotorDegrees) || absoluteMotorDegrees < 0
                || absoluteMotorDegrees > 360 || !Double.isFinite(maxCorrectionDegrees)
                || maxCorrectionDegrees < 0 || maxCorrectionDegrees >= 180.0 / gearRatio) return false;
        double relativeDegrees = positionDegrees(relativeTicks);
        double correction = estimateDegrees(absoluteMotorDegrees, relativeDegrees) - relativeDegrees;
        if (!Double.isFinite(correction) || Math.abs(correction) > maxCorrectionDegrees + 1e-9) return false;
        initialDegrees += correction;
        return true;
    }

    public double velocityDegreesPerSecond(double ticksPerSecond) {
        return ticksPerSecond * degreesPerTick;
    }
}
