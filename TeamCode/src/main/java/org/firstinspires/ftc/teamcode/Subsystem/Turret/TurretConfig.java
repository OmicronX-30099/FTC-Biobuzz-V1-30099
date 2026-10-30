package org.firstinspires.ftc.teamcode.Subsystem.Turret;


/** Shared hardware calibration and tuned controller settings for the robot turret. */
public final class TurretConfig {
    private TurretConfig() { }

    // Hardware/calibration changes take effect at INIT.
    public static String MOTOR_NAME = "turret";
    public static String ABSOLUTE_ENCODER_NAME = "absEn";
    public static String RELATIVE_ENCODER_NAME = "frDrive";
    public static boolean MOTOR_REVERSED = false;
    public static boolean RELATIVE_ENCODER_REVERSED = false;
    public static boolean ABSOLUTE_ENCODER_REVERSED = true;
    public static double TICKS_PER_MOTOR_REV = 384.5;
    public static double MOTOR_REVS_PER_TURRET_REV = 4.5;
    public static double ABSOLUTE_FULL_SCALE_VOLTS = 3.3;
    public static double ABSOLUTE_ZERO_DEG = 0.0;
    // Selects the starting revolution of the motor-side absolute encoder.
    // Keep this within 40 turret degrees of the physical starting angle.
    public static double INITIAL_TURRET_ESTIMATE_DEG = 0.0;
    // Backup calibration uses the nearest absolute revolution; the absolute pattern repeats every 80°.
    public static volatile double BACKUP_MAX_CORRECTION_DEG = 15.0;

    // The calibrated turret angle increases clockwise; Pedro heading increases counterclockwise.
    public static volatile boolean CLOCKWISE_POSITIVE = true;

    public static volatile double kP = 0.035;
    public static volatile double kD = 0.001;
    public static volatile double kS = 0.05;
    public static volatile double DEADBAND_DEG = 0.21;
    public static volatile double FRICTION_RAMP_DEG = 5.0;
    public static volatile double MAX_POWER = 1.0;
    public static volatile double MIN_ANGLE_DEG = -230.0;
    public static volatile double MAX_ANGLE_DEG = 270.0;
}
