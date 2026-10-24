package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController;
import org.firstinspires.ftc.teamcode.control.ShooterRecoveryTimer;
import org.firstinspires.ftc.teamcode.Util.ServoPositions;

/**
 * Follows the direct init/loop structure of 16010-Worlds' testing OpModes.
 * Tune the target velocity and controller gains live from the FTC Dashboard config page.
 * Uses a software P + static/velocity feedforward controller.
 */
@Config
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends OpMode {
    // Hardware names are read at INIT. All other controls are read every loop.
    public static String MOTOR_NAME = "shooter";
    public static String LEFT_INTAKE_NAME = "left_intake";
    public static String RIGHT_INTAKE_NAME = "right_intake";
    public static String HOOD_NAME = "hood";
    public static volatile double TARGET_VELOCITY_TPS = 1500.0;
    public static volatile double INTAKE_FEED_POWER = 0.7;
    public static volatile double HOOD_POSITION = ServoPositions.HOOD_DEFAULT;

    // Starting gains from FTC-Decode-v3; tune them for this season's shooter.
    public static volatile double kP = ShooterVelocityController.DEFAULT_KP;
    public static volatile double kS = ShooterVelocityController.DEFAULT_KS;
    public static volatile double kV = ShooterVelocityController.DEFAULT_KV;

    // Recovery is inferred from a dip after the shooter has settled at its target.
    public static volatile double RECOVERY_TOLERANCE_TPS = 20.0;
    public static volatile double SHOT_DROP_TPS = 50.0;
    public static volatile double RECOVERY_SETTLE_MS = 20.0;

    private DcMotorEx flywheel;
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private Servo hood;
    private MultipleTelemetry telemetryM;
    private FtcDashboard dashboard;
    private final ShooterRecoveryTimer recoveryTimer = new ShooterRecoveryTimer();

    @Override
    public void init() {
        recoveryTimer.reset();
        flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        leftIntake = hardwareMap.get(DcMotorEx.class, LEFT_INTAKE_NAME);
        rightIntake = hardwareMap.get(DcMotorEx.class, RIGHT_INTAKE_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        flywheel.setPower(0.0);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Read the encoder, but compute velocity control here rather than in the hub.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMotorEnable();
        configureIntake(leftIntake, DcMotorSimple.Direction.FORWARD);
        configureIntake(rightIntake, DcMotorSimple.Direction.REVERSE);
        if (hood instanceof PwmControl) {
            ((PwmControl) hood).setPwmEnable();
        } else {
            hood.getController().pwmEnable();
        }

        dashboard = FtcDashboard.getInstance();
        telemetryM = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetryM.setMsTransmissionInterval(50);
        showInitTelemetry();
    }

    private void configureIntake(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMotorEnable();
    }

    @Override
    public void init_loop() {
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetryM.addData("Test version", "Shooter Test v7");
        telemetryM.addLine("Press START to command shooter; hold A to feed intake.");
        telemetryM.addLine("Tune target velocity and gains from the Dashboard config page.");
        telemetryM.addData("Motor connection", flywheel.getConnectionInfo());
        telemetryM.update();
    }

    @Override
    public void loop() {
        long loopStartNs = System.nanoTime();

        // Command the hood directly each loop, like the reference's servo tests.
        hood.setPosition(ServoPositions.clipHoodPosition(HOOD_POSITION));

        double velocity = flywheel.getVelocity();
        double target = TARGET_VELOCITY_TPS;
        double p = kP;
        double s = kS;
        double v = kV;
        double power = ShooterVelocityController.calculate(target, velocity, p, s, v);
        flywheel.setPower(power);

        if (gamepad1.right_trigger > 0.1) {
            leftIntake.setPower(INTAKE_FEED_POWER);
            rightIntake.setPower(INTAKE_FEED_POWER);
        } else {
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }

        double nowSeconds = System.nanoTime() / 1e9;
        recoveryTimer.update(nowSeconds, target, velocity,
                RECOVERY_TOLERANCE_TPS, SHOT_DROP_TPS, RECOVERY_SETTLE_MS);

        telemetryM.addData("Test version", "Shooter Test v7");
        telemetryM.addData("Motor mode", flywheel.getMode());
        telemetryM.addData("Current velocity (ticks/s)", velocity);
        telemetryM.addData("Target velocity (ticks/s)", target);
        telemetryM.addData("Loop time (ms)", (System.nanoTime() - loopStartNs) / 1e6);
        telemetryM.addData("Recovery status", recoveryTimer.getState());
        telemetryM.addData("Recovery elapsed (ms)", recoveryTimer.getElapsedMs(nowSeconds));
        telemetryM.addData("Last recovery (ms)", recoveryTimer.getLastRecoveryMs());
        telemetryM.addData("Last velocity drop (ticks/s)", recoveryTimer.getLastDropTPS());
        telemetryM.addData("Completed recoveries", recoveryTimer.getCompletedRecoveries());
        if (recoveryTimer.getState() == ShooterRecoveryTimer.State.INVALID_INPUT) {
            telemetryM.addLine("Recovery settings: use finite values, drop > tolerance >= 0, settle >= 0.");
        }
        telemetryM.addData("Motor current (A)", flywheel.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Motor hardware enabled", flywheel.isMotorEnabled());
        telemetryM.update();

        // Keep fixed series names so Dashboard can graph target and measured velocity over time.
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target velocity (ticks/s)", target);
        packet.put("Actual velocity (ticks/s)", velocity);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setPower(0.0);
        }
        if (leftIntake != null) {
            leftIntake.setPower(0.0);
        }
        if (rightIntake != null) {
            rightIntake.setPower(0.0);
        }
    }
}
