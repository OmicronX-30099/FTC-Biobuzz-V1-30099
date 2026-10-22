package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.TurretPositionController;
import org.firstinspires.ftc.teamcode.control.TurretEncoderTracker;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConfig;

/** Motor turret tuning using the existing absEn and frDrive encoder wiring. */
@TeleOp(name = "Turret Test", group = "Tests")
public class TurretTest extends OpMode {
    // Hardware/calibration settings are captured at INIT; restart INIT after changing them.
    public static String MOTOR_NAME = TurretConfig.MOTOR_NAME;
    public static String ABSOLUTE_ENCODER_NAME = TurretConfig.ABSOLUTE_ENCODER_NAME;
    public static String RELATIVE_ENCODER_NAME = TurretConfig.RELATIVE_ENCODER_NAME;
    public static boolean MOTOR_REVERSED = TurretConfig.MOTOR_REVERSED;
    public static boolean RELATIVE_ENCODER_REVERSED = TurretConfig.RELATIVE_ENCODER_REVERSED;
    public static boolean ABSOLUTE_ENCODER_REVERSED = TurretConfig.ABSOLUTE_ENCODER_REVERSED;
    public static double TICKS_PER_MOTOR_REV = TurretConfig.TICKS_PER_MOTOR_REV;
    public static double MOTOR_REVS_PER_TURRET_REV = TurretConfig.MOTOR_REVS_PER_TURRET_REV;
    public static double ABSOLUTE_FULL_SCALE_VOLTS = TurretConfig.ABSOLUTE_FULL_SCALE_VOLTS;
    public static double ABSOLUTE_ZERO_DEG = TurretConfig.ABSOLUTE_ZERO_DEG;
    // A motor-side absolute encoder repeats every 80 turret degrees at 4.5:1.
    // Enter an approximate starting turret angle to select the correct revolution.
    public static double INITIAL_TURRET_ESTIMATE_DEG = TurretConfig.INITIAL_TURRET_ESTIMATE_DEG;

    // Source-defined settings. Gains are motor-power units, not REV Hub PIDF gains.
    public static volatile double TARGET_DEG = 0.0;
    public static volatile double kP = TurretConfig.kP;
    public static volatile double kD = TurretConfig.kD;
    public static volatile double kS = TurretConfig.kS;
    public static volatile double FRICTION_RAMP_DEG = TurretConfig.FRICTION_RAMP_DEG;
    public static volatile double DEADBAND_DEG = TurretConfig.DEADBAND_DEG;
    public static volatile double MAX_POWER = TurretConfig.MAX_POWER;
    public static volatile double MIN_ANGLE_DEG = TurretConfig.MIN_ANGLE_DEG;
    public static volatile double MAX_ANGLE_DEG = TurretConfig.MAX_ANGLE_DEG;

    private DcMotorEx motor;
    private DcMotorEx relativeEncoder;
    private AnalogInput absoluteEncoder;
    private VoltageSensor battery;
    private TurretEncoderTracker tracker;
    private double fullScaleVolts;
    private double initialEstimate;
    private double absoluteVoltage;
    private double absoluteDegrees;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        if (!Double.isFinite(TICKS_PER_MOTOR_REV) || TICKS_PER_MOTOR_REV <= 0
                || !Double.isFinite(MOTOR_REVS_PER_TURRET_REV) || MOTOR_REVS_PER_TURRET_REV <= 0
                || !Double.isFinite(ABSOLUTE_FULL_SCALE_VOLTS) || ABSOLUTE_FULL_SCALE_VOLTS <= 0
                || !Double.isFinite(ABSOLUTE_ZERO_DEG) || !Double.isFinite(INITIAL_TURRET_ESTIMATE_DEG)) {
            throw new IllegalArgumentException("Turret encoder calibration must be finite, with positive ratios and voltage scale.");
        }

        fullScaleVolts = ABSOLUTE_FULL_SCALE_VOLTS;
        initialEstimate = INITIAL_TURRET_ESTIMATE_DEG;
        tracker = new TurretEncoderTracker(TICKS_PER_MOTOR_REV, MOTOR_REVS_PER_TURRET_REV,
                ABSOLUTE_ZERO_DEG, ABSOLUTE_ENCODER_REVERSED, RELATIVE_ENCODER_REVERSED);

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        relativeEncoder = hardwareMap.get(DcMotorEx.class, RELATIVE_ENCODER_NAME);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, ABSOLUTE_ENCODER_NAME);
        motor.setPower(0);
        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // frDrive is an encoder input here: never reset it or command its motor output.
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        showInitTelemetry();
    }

    private void readAbsoluteEncoder() {
        absoluteVoltage = absoluteEncoder.getVoltage();
        absoluteDegrees = absoluteVoltage / fullScaleVolts * 360.0;
    }

    private double estimateInitialDegrees() {
        return tracker.estimateDegrees(absoluteDegrees, initialEstimate);
    }

    private void showInitTelemetry() {
        readAbsoluteEncoder();
        telemetry.addLine("Press START to run TARGET_DEG and gains configured in TurretTest.java.");
        telemetry.addLine("Motor-side absolute repeats every 80 turret degrees; check the initial estimate.");
        telemetry.addData("Absolute voltage (V)", absoluteVoltage);
        telemetry.addData("Raw absolute motor angle (deg)", absoluteDegrees);
        telemetry.addData("Estimated starting turret angle (deg)", estimateInitialDegrees());
        telemetry.addData("Relative encoder ticks", relativeEncoder.getCurrentPosition());
        telemetry.addData("Turret motor", motor.getConnectionInfo());
        telemetry.addData("Relative encoder input", relativeEncoder.getConnectionInfo());
        telemetry.update();
    }

    @Override
    public void start() {
        // Seed at START so manually moving the turret during INIT is accounted for.
        readAbsoluteEncoder();
        tracker.seed(absoluteDegrees, relativeEncoder.getCurrentPosition(), initialEstimate);
    }

    @Override
    public void loop() {
        readAbsoluteEncoder();
        int ticks = relativeEncoder.getCurrentPosition();
        double angle = tracker.positionDegrees(ticks);
        double velocity = tracker.velocityDegreesPerSecond(relativeEncoder.getVelocity());
        double voltage = battery == null ? 12.0 : battery.getVoltage();
        double minAngle = MIN_ANGLE_DEG;
        double maxAngle = MAX_ANGLE_DEG;
        boolean validLimits = Double.isFinite(minAngle) && Double.isFinite(maxAngle) && minAngle < maxAngle;
        double requestedTarget = TARGET_DEG;
        double target = validLimits ? Math.max(minAngle, Math.min(maxAngle, requestedTarget)) : angle;
        double p = kP;
        double d = kD;
        double s = kS;
        double ramp = FRICTION_RAMP_DEG;
        double deadband = DEADBAND_DEG;
        double maxPower = MAX_POWER;
        double power = validLimits ? TurretPositionController.calculate(target, angle, velocity,
                p, d, s, ramp, deadband, maxPower, voltage) : 0.0;
        if ((angle <= minAngle && power < 0) || (angle >= maxAngle && power > 0)) power = 0;
        motor.setPower(power);

        telemetry.addData("Absolute voltage (V)", absoluteVoltage);
        telemetry.addData("Raw absolute motor angle (deg)", absoluteDegrees);
        telemetry.addData("Relative encoder ticks", ticks);
        telemetry.addData("Turret angle (deg)", angle);
        telemetry.addData("Turret velocity (deg/s)", velocity);
        telemetry.addData("Requested target (deg)", requestedTarget);
        telemetry.addData("Limited target (deg)", target);
        telemetry.addData("Position error (deg)", target - angle);
        telemetry.addData("P contribution", p * (target - angle));
        telemetry.addData("D contribution", -d * velocity);
        telemetry.addData("Friction contribution", s * Math.signum(target - angle)
                * Math.min(Math.abs(target - angle) / ramp, 1.0));
        telemetry.addData("Motor power", power);
        telemetry.addData("Battery voltage (V)", voltage);
        telemetry.addData("kP / kD / kS", "%.5f / %.5f / %.5f", p, d, s);
        telemetry.addData("Angle limits valid", validLimits);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motor != null) motor.setPower(0);
    }
}
