package org.firstinspires.ftc.teamcode.Subsystem.Turret;


/** Paired measured shots, sorted by distance. Recomputed from robot X/Y for confirmed goal (56, 59). */
public final class ShotCalibration {
    private ShotCalibration() { }

    // Keep matching indices paired and distances strictly increasing when editing this table.
    public static volatile double[] DISTANCE_INCHES = {
            36.12014673281381, 51.440559872536376,
            56.88717693821693, 70.41114968525936
    };
    public static volatile double[] VELOCITY_TPS = {1440, 1480, 1540, 1600};
    public static volatile double[] HOOD_POSITIONS = {0.485, 0.30, 0.25, 0.25};
}
