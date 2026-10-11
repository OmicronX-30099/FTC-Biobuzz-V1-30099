package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController;
import org.firstinspires.ftc.teamcode.control.ShooterRecoveryTimer;

/**
 * Follows the direct init/loop structure of 16010-Worlds' testing OpModes.
 * Press START, then edit ShooterTest in Dashboard and save the changes.
 * Uses last season's software P + static/velocity feedforward controller.
 */
@TeleOp(name = "Shooter Test", group = "Tests")
@Config
public class ShooterTest extends OpMode {
    // Hardware names are read at INIT. All other controls are read every loop.
    public static String MOTOR_NAME = "shooter";
    public static String HOOD_NAME = "hood";
    public static volatile double TARGET_VELOCITY_TPS = 1500.0;
    public static volatile double HOOD_POSITION = 0.5;

    // Starting gains from FTC-Decode-v3; tune them for this season's shooter.
    public static volatile double kP = ShooterVelocityController.DEFAULT_KP;
    public static volatile double kS = ShooterVelocityController.DEFAULT_KS;
    public static volatile double kV = ShooterVelocityController.DEFAULT_KV;

    // Recovery is inferred from a dip after the shooter has settled at its target.
    public static volatile double RECOVERY_TOLERANCE_TPS = 20.0;
    public static volatile double SHOT_DROP_TPS = 50.0;
    public static volatile double RECOVERY_SETTLE_MS = 20.0;

    private DcMotorEx flywheel;
    private Servo hood;
    private MultipleTelemetry telemetryM;
    private final ShooterRecoveryTimer recoveryTimer = new ShooterRecoveryTimer();

    @Override
    public void init() {
        recoveryTimer.reset();
        flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_NAME);
        flywheel.setPower(0.0);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Read the encoder, but compute velocity control here rather than in the hub.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMotorEnable();
        if (hood instanceof PwmControl) {
            ((PwmControl) hood).setPwmEnable();
        } else {
            hood.getController().pwmEnable();
        }

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM.setMsTransmissionInterval(50);
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetryM.addData("Test version", "Shooter Test v6");
        telemetryM.addLine("Press START to command the motor and hood.");
        telemetryM.addLine("Change TARGET_VELOCITY_TPS, HOOD_POSITION, kP, kS, and kV in Dashboard.");
        telemetryM.addData("Motor connection", flywheel.getConnectionInfo());
        telemetryM.addData("Hood connection", hood.getConnectionInfo());
        telemetryM.update();
    }

    @Override
    public void loop() {
        // Command the hood directly each loop, like the reference's servo tests.
        hood.setPosition(Range.clip(HOOD_POSITION, 0.0, 1.0));

        double velocity = flywheel.getVelocity();
        double target = TARGET_VELOCITY_TPS;
        double p = kP;
        double s = kS;
        double v = kV;
        double power = ShooterVelocityController.calculate(target, velocity, p, s, v);
        flywheel.setPower(power);

        double nowSeconds = System.nanoTime() / 1e9;
        recoveryTimer.update(nowSeconds, target, velocity,
                RECOVERY_TOLERANCE_TPS, SHOT_DROP_TPS, RECOVERY_SETTLE_MS);

        telemetryM.addData("Test version", "Shooter Test v6");
        telemetryM.addData("Motor mode", flywheel.getMode());
        telemetryM.addData("Current velocity (ticks/s)", velocity);
        telemetryM.addData("Target velocity (ticks/s)", target);
        telemetryM.addData("Velocity error (ticks/s)", target - velocity);
        telemetryM.addData("Motor power", power);
        telemetryM.addData("kP", p);
        telemetryM.addData("kS", s);
        telemetryM.addData("kV", v);
        telemetryM.addData("Recovery status", recoveryTimer.getState());
        telemetryM.addData("Recovery elapsed (ms)", recoveryTimer.getElapsedMs(nowSeconds));
        telemetryM.addData("Last recovery (ms)", recoveryTimer.getLastRecoveryMs());
        telemetryM.addData("Last velocity drop (ticks/s)", recoveryTimer.getLastDropTPS());
        telemetryM.addData("Completed recoveries", recoveryTimer.getCompletedRecoveries());
        if (recoveryTimer.getState() == ShooterRecoveryTimer.State.INVALID_INPUT) {
            telemetryM.addLine("Recovery settings: use finite values, drop > tolerance >= 0, settle >= 0.");
        }
        telemetryM.addData("Motor current (A)", flywheel.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Hood requested position", HOOD_POSITION);
        // This is the last commanded position, not physical servo feedback.
        telemetryM.addData("Hood commanded position", hood.getPosition());
        telemetryM.addData("Motor hardware enabled", flywheel.isMotorEnabled());
        if (hood instanceof PwmControl) {
            telemetryM.addData("Hood PWM enabled", ((PwmControl) hood).isPwmEnabled());
        }
        telemetryM.update();
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setPower(0.0);
        }
    }
}
