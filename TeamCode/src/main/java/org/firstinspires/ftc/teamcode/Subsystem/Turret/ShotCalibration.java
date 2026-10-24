package org.firstinspires.ftc.teamcode.Subsystem.Turret;


/** Paired measured shots sorted by distance from the selected goal, in inches. */
public final class ShotCalibration {
    private ShotCalibration() { }

    // Keep matching indices paired and distances strictly increasing when editing this table.
    public static volatile double[] DISTANCE_INCHES = {
            23, 28, 38, 48, 58, 68
    };
    public static volatile double[] VELOCITY_TPS = {1380, 1420, 1410, 1480, 1560, 1580};
    public static volatile double[] HOOD_POSITIONS = {0.485, 0.45, 0.30, 0.25, 0.20, 0.20};
}
