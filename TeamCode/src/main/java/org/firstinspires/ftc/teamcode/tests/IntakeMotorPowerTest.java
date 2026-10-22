package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Commands power directly and reports encoder velocity and current to Dashboard. */
@TeleOp(name = "Intake Motor Power Test", group = "Tests")
public class IntakeMotorPowerTest extends LinearOpMode {
    // Matches Rollers and IntakeTest. Names and directions are read at INIT.
    public static String LEFT_MOTOR_NAME = "left_intake";
    public static String RIGHT_MOTOR_NAME = "right_intake";
    public static boolean LEFT_MOTOR_REVERSED = false;
    public static boolean RIGHT_MOTOR_REVERSED = true;
    // Read every loop; positive/negative power selects the motor's direction.
    public static volatile double MOTOR_POWER = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftIntake = hardwareMap.get(DcMotorEx.class, LEFT_MOTOR_NAME);
        DcMotorEx rightIntake = hardwareMap.get(DcMotorEx.class, RIGHT_MOTOR_NAME);
        leftIntake.setPower(0.0);
        rightIntake.setPower(0.0);
        if (leftIntake == rightIntake) {
            throw new IllegalArgumentException("Left and right intake names must select different motors.");
        }
        configureMotor(leftIntake, LEFT_MOTOR_REVERSED);
        configureMotor(rightIntake, RIGHT_MOTOR_REVERSED);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        try {
            telemetry.addLine("Press START to run MOTOR_POWER from IntakeMotorPowerTest.java (-1 to 1; zero stops).");
            telemetry.addData("Left motor name", LEFT_MOTOR_NAME);
            telemetry.addData("Left motor connection", leftIntake.getConnectionInfo());
            telemetry.addData("Left motor direction", leftIntake.getDirection());
            telemetry.addData("Right motor name", RIGHT_MOTOR_NAME);
            telemetry.addData("Right motor connection", rightIntake.getConnectionInfo());
            telemetry.addData("Right motor direction", rightIntake.getDirection());
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                double requestedPower = MOTOR_POWER;
                double power = Double.isFinite(requestedPower)
                        ? Range.clip(requestedPower, -1.0, 1.0) : 0.0;
                leftIntake.setPower(power);
                rightIntake.setPower(power);

                telemetry.addData("Motor power", power);
                telemetry.addData("Left velocity (ticks/s)", leftIntake.getVelocity());
                telemetry.addData("Left current (A)", leftIntake.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Right velocity (ticks/s)", rightIntake.getVelocity());
                telemetry.addData("Right current (A)", rightIntake.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
                idle();
            }
        } finally {
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }
    }

    private void configureMotor(DcMotorEx motor, boolean reversed) {
        motor.setDirection(reversed
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Use raw power while retaining encoder readings for telemetry.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMotorEnable();
    }
}
