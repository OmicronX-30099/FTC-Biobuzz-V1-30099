package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.math.Pose;

/** Separate translation and turret-response leads; neither uses projectile time of flight. */
public final class ShootingPosePrediction {
    public static final double LEAD_SECONDS = 0.7;
    public static final double ANGULAR_LEAD_SECONDS = 0.05;

    private ShootingPosePrediction() { }

    public static Pose predict(Pose current, double fieldVelocityX, double fieldVelocityY) {
        return predict(current, fieldVelocityX, fieldVelocityY, 0);
    }

    /** Angular velocity is counterclockwise-positive radians/second, matching Pedro heading. */
    public static Pose predict(Pose current, double fieldVelocityX, double fieldVelocityY,
                               double angularVelocityRadPerSec) {
        if (fieldVelocityX == 0 && fieldVelocityY == 0 && angularVelocityRadPerSec == 0) return current;
        return new Pose(current.x() + fieldVelocityX * LEAD_SECONDS,
                current.y() + fieldVelocityY * LEAD_SECONDS,
                current.heading() + angularVelocityRadPerSec * ANGULAR_LEAD_SECONDS);
    }
}
