package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.math.Pose;

/** Predicts a robot pose from measured field-relative linear and angular velocity. */
public final class ShootingPosePrediction {
    public static final double MIN_FIELD_COORDINATE_IN = 7.0;
    public static final double MAX_FIELD_COORDINATE_IN = 141.5 - MIN_FIELD_COORDINATE_IN;
    /** Legacy default retained for callers that do not yet supply projectile flight time. */
    public static final double LEAD_SECONDS = 0.7;
    public static final double ANGULAR_LEAD_SECONDS = 0.05;

    private ShootingPosePrediction() { }

    public static Pose predict(Pose current, double fieldVelocityX, double fieldVelocityY) {
        return predict(current, fieldVelocityX, fieldVelocityY, 0, LEAD_SECONDS);
    }

    /** Angular velocity is counterclockwise-positive radians/second, matching Pedro heading. */
    public static Pose predict(Pose current, double fieldVelocityX, double fieldVelocityY,
                               double angularVelocityRadPerSec) {
        if (current == null || !Double.isFinite(fieldVelocityX) || !Double.isFinite(fieldVelocityY)
                || !Double.isFinite(angularVelocityRadPerSec)) return clampCurrent(current);
        return predictedPose(current, fieldVelocityX * LEAD_SECONDS, fieldVelocityY * LEAD_SECONDS,
                angularVelocityRadPerSec * ANGULAR_LEAD_SECONDS);
    }

    /** Leads translation by projectile flight time and heading by the short turret response horizon. */
    public static Pose predict(Pose current, double fieldVelocityX, double fieldVelocityY,
                               double angularVelocityRadPerSec, double flightTimeSeconds) {
        if (current == null || !Double.isFinite(fieldVelocityX) || !Double.isFinite(fieldVelocityY)
                || !Double.isFinite(angularVelocityRadPerSec) || !Double.isFinite(flightTimeSeconds)
                || flightTimeSeconds < 0) return clampCurrent(current);
        return predictedPose(current, fieldVelocityX * flightTimeSeconds, fieldVelocityY * flightTimeSeconds,
                angularVelocityRadPerSec * ANGULAR_LEAD_SECONDS);
    }

    private static Pose predictedPose(Pose current, double deltaX, double deltaY, double deltaHeading) {
        double x = clamp(current.x() + deltaX);
        double y = clamp(current.y() + deltaY);
        double heading = current.heading() + deltaHeading;
        // Reuse the measured pose when motion is zero or the boundary clamp removes it.
        if (x == current.x() && y == current.y() && heading == current.heading()) return current;
        return new Pose(x, y, heading);
    }

    private static double clamp(double coordinate) {
        return Math.max(MIN_FIELD_COORDINATE_IN, Math.min(MAX_FIELD_COORDINATE_IN, coordinate));
    }

    private static Pose clampCurrent(Pose current) {
        return current == null ? null : predictedPose(current, 0, 0, 0);
    }
}
