package org.firstinspires.ftc.teamcode.control;

/** Aims from a supplied measured/predicted robot position and heading. */
public final class TurretAiming {
    private TurretAiming() { }

    public static double targetDegrees(double robotX, double robotY, double headingRadians,
                                       double goalX, double goalY, double currentDegrees,
                                       double minDegrees, double maxDegrees, boolean clockwisePositive) {
        return targetDegrees(robotX, robotY, headingRadians, goalX, goalY, currentDegrees,
                minDegrees, maxDegrees, clockwisePositive, 0);
    }

    /** Positive trim turns physically right, independent of the encoder sign convention. */
    public static double targetDegrees(double robotX, double robotY, double headingRadians,
                                       double goalX, double goalY, double currentDegrees,
                                       double minDegrees, double maxDegrees, boolean clockwisePositive,
                                       double rightTrimDegrees) {
        if (!Double.isFinite(robotX) || !Double.isFinite(robotY) || !Double.isFinite(headingRadians)
                || !Double.isFinite(goalX) || !Double.isFinite(goalY) || !Double.isFinite(currentDegrees)
                || !Double.isFinite(minDegrees) || !Double.isFinite(maxDegrees) || minDegrees >= maxDegrees
                || !Double.isFinite(rightTrimDegrees)) {
            return Double.NaN;
        }
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        if (dx == 0 && dy == 0) return Math.max(minDegrees, Math.min(maxDegrees, currentDegrees));

        // Pedro heading is counterclockwise from field +X; turret zero faces robot forward.
        // Convert the bearing into the encoder's convention before choosing a valid revolution.
        double desired = Math.toDegrees(Math.atan2(dy, dx) - headingRadians);
        if (clockwisePositive) desired = -desired;
        desired += clockwisePositive ? rightTrimDegrees : -rightTrimDegrees;
        desired = ((desired + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        double minTurn = Math.ceil((minDegrees - desired) / 360.0);
        double maxTurn = Math.floor((maxDegrees - desired) / 360.0);
        if (minTurn > maxTurn) return Math.max(minDegrees, Math.min(maxDegrees, desired));

        double nearestTurn = Math.round((currentDegrees - desired) / 360.0);
        nearestTurn = Math.max(minTurn, Math.min(maxTurn, nearestTurn));
        return desired + 360.0 * nearestTurn;
    }
}
