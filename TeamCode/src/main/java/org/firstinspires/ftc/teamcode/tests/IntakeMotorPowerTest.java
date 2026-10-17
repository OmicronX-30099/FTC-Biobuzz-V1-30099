package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Commands power directly and reports encoder velocity and current to Dashboard. */
@Config
@TeleOp(name = "Intake Motor Power Test", group = "Tests")
public class IntakeMotorPowerTest extends LinearOpMode {
    // Matches IntakeTest's current hardware mapping. Name and direction are read at INIT.
    public static String MOTOR_NAME = "shooter";
    public static boolean MOTOR_REVERSED = false;
    // Read every loop; positive/negative power selects the motor's direction.
    public static volatile double MOTOR_POWER = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        intake.setPower(0.0);
        intake.setDirection(MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Use raw power while retaining encoder readings for telemetry.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMotorEnable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        try {
            telemetry.addLine("Press START, then change MOTOR_POWER in Dashboard (-1 to 1; zero stops).");
            telemetry.addData("Motor name", MOTOR_NAME);
            telemetry.addData("Motor connection", intake.getConnectionInfo());
            telemetry.addData("Motor direction", intake.getDirection());
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                double requestedPower = MOTOR_POWER;
                double power = Double.isFinite(requestedPower)
                        ? Range.clip(requestedPower, -1.0, 1.0) : 0.0;
                intake.setPower(power);

                telemetry.addData("Motor power", power);
                telemetry.addData("Intake velocity (ticks/s)", intake.getVelocity());
                telemetry.addData("Motor current (A)", intake.getCurrent(CurrentUnit.AMPS));
                telemetry.update();
                idle();
            }
        } finally {
            intake.setPower(0.0);
        }
    }
}
