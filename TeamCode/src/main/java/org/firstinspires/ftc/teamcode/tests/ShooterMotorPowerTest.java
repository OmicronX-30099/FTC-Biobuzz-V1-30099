package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Shooter Motor Power Test", group = "Tests")
public class ShooterMotorPowerTest extends LinearOpMode {
    public static volatile double MOTOR_POWER = 0.1;
    public static volatile double HOOD_POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Servo hood = hardwareMap.get(Servo.class, "hood");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter.setPower(0.0);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                double motorPower = Range.clip(MOTOR_POWER, -1.0, 1.0);
                double hoodPosition = Range.clip(HOOD_POSITION, 0.0, 1.0);
                shooter.setPower(motorPower);
                hood.setPosition(hoodPosition);
                telemetry.addData("Motor power", motorPower);
                telemetry.addData("Hood position command", hoodPosition);
                telemetry.addData("Shooter velocity (ticks/s)", shooter.getVelocity());
                telemetry.update();
                idle();
            }
        } finally {
            shooter.setPower(0.0);
        }
    }
}
