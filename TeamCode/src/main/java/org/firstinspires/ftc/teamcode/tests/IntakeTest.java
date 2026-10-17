package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.IntakeVelocityController;

/** Press START, then tune the signed target and gains under IntakeTest in Dashboard. */
@TeleOp(name = "Intake Test", group = "Tests")
@Config
public class IntakeTest extends OpMode {
    // Hardware name and direction are read at INIT; target and gains are read every loop.
    public static String MOTOR_NAME = "shooter";
    public static boolean MOTOR_REVERSED = false;
    public static volatile double TARGET_VELOCITY_TPS = 0.0;

    // Starting estimates only; tune these independently of the shooter.
    public static volatile double kP = IntakeVelocityController.DEFAULT_KP;
    public static volatile double kS = IntakeVelocityController.DEFAULT_KS;
    public static volatile double kV = IntakeVelocityController.DEFAULT_KV;

    private DcMotorEx intake;
    private MultipleTelemetry telemetryM;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        intake.setPower(0.0);
        intake.setDirection(MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Encoder readings remain available; this software controller computes motor power.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMotorEnable();

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM.setMsTransmissionInterval(50);
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetryM.addLine("Press START, then change TARGET_VELOCITY_TPS, kP, kS, and kV in Dashboard.");
        telemetryM.addLine("Positive target collects; negative ejects; zero stops. Configure motor direction accordingly.");
        telemetryM.addData("Motor connection", intake.getConnectionInfo());
        telemetryM.addData("Motor direction", intake.getDirection());
        telemetryM.addData("Target velocity (ticks/s)", TARGET_VELOCITY_TPS);
        telemetryM.update();
    }

    @Override
    public void loop() {
        double velocity = intake.getVelocity();
        double target = TARGET_VELOCITY_TPS;
        double p = kP;
        double s = kS;
        double v = kV;
        double power = IntakeVelocityController.calculate(target, velocity, p, s, v);
        intake.setPower(power);

        telemetryM.addData("Motor mode", intake.getMode());
        telemetryM.addData("Current velocity (ticks/s)", velocity);
        telemetryM.addData("Target velocity (ticks/s)", target);
        telemetryM.addData("Velocity error (ticks/s)", target - velocity);
        telemetryM.addData("Motor power", power);
        telemetryM.addData("Motor current (A)", intake.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("kP", p);
        telemetryM.addData("kS", s);
        telemetryM.addData("kV", v);
        telemetryM.update();
    }

    @Override
    public void stop() {
        if (intake != null) {
            intake.setPower(0.0);
        }
    }
}
