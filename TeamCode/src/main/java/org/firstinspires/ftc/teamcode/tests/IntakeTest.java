package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.control.IntakeVelocityController;

/** Edit the signed target and gains in source before building and running this test. */
@TeleOp(name = "Intake Test", group = "Tests")
public class IntakeTest extends OpMode {
    // Names and directions match Rollers and are read at INIT.
    public static String LEFT_MOTOR_NAME = "left_intake";
    public static String RIGHT_MOTOR_NAME = "right_intake";
    public static boolean LEFT_MOTOR_REVERSED = false;
    public static boolean RIGHT_MOTOR_REVERSED = true;
    public static volatile double TARGET_VELOCITY_TPS = 1100.0;

    // Tuned intake defaults, shared with Rollers.
    public static volatile double kP = IntakeVelocityController.DEFAULT_KP;
    public static volatile double kS = IntakeVelocityController.DEFAULT_KS;
    public static volatile double kV = IntakeVelocityController.DEFAULT_KV;

    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private MultipleTelemetry telemetryM;

    @Override
    public void init() {
        leftIntake = hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        rightIntake = hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
        if (leftIntake == rightIntake) {
            throw new IllegalArgumentException("Left and right intake names must select different motors.");
        }
        configureMotor(leftIntake, LEFT_MOTOR_REVERSED);
        configureMotor(rightIntake, RIGHT_MOTOR_REVERSED);

        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM.setMsTransmissionInterval(50);
        showInitTelemetry();
    }

    private void configureMotor(DcMotorEx motor, boolean reversed) {
        motor.setDirection(reversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Encoder readings remain available; this software controller computes motor power.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMotorEnable();
    }

    @Override
    public void init_loop() {
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetryM.addLine("Press START to run the target/gains configured in IntakeTest.java.");
        telemetryM.addLine("Positive target collects; negative ejects; zero stops. Configure motor direction accordingly.");
        telemetryM.addLine("Left motor TPS supplies feedback; both motors receive the same power.");
        telemetryM.addData("Left motor connection", leftIntake.getConnectionInfo());
        telemetryM.addData("Left motor direction", leftIntake.getDirection());
        telemetryM.addData("Right motor connection", rightIntake.getConnectionInfo());
        telemetryM.addData("Right motor direction", rightIntake.getDirection());
        telemetryM.addData("Target velocity (ticks/s)", TARGET_VELOCITY_TPS);
        telemetryM.update();
    }

    @Override
    public void loop() {
        double leftVelocity = leftIntake.getVelocity();
        double target = TARGET_VELOCITY_TPS;
        double p = kP;
        double s = kS;
        double v = kV;
        double power = IntakeVelocityController.calculate(target, leftVelocity, p, s, v);
        leftIntake.setPower(power);
        rightIntake.setPower(power);

        telemetryM.addData("Target velocity (ticks/s)", target);
        telemetryM.addData("Left velocity (ticks/s)", leftVelocity);
        telemetryM.addData("Left error (ticks/s)", target - leftVelocity);
        telemetryM.addData("Motor power (both)", power);
        telemetryM.addData("Left current (A)", leftIntake.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Right current (A)", rightIntake.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("kP", p);
        telemetryM.addData("kS", s);
        telemetryM.addData("kV", v);
        telemetryM.update();
    }

    @Override
    public void stop() {
        if (leftIntake != null) leftIntake.setPower(0.0);
        if (rightIntake != null) rightIntake.setPower(0.0);
    }
}
