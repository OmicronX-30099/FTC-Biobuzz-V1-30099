package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.ShooterVelocityController;

/** Standalone velocity test for the two shooter motors. */
@Config
@TeleOp(name = "Dual Shooter Velocity Test", group = "Tests")
public class DualShooterVelocityTest extends LinearOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 100;
    private static final long TELEMETRY_INTERVAL_NS = TELEMETRY_INTERVAL_MS * 1_000_000L;

    // Set each target from Dashboard. Zero keeps that motor stopped.
    public static volatile double BACK_TARGET_TPS = 0.0;
    public static volatile double BACK_KP = ShooterVelocityController.DEFAULT_KP;
    public static volatile double BACK_KS = ShooterVelocityController.DEFAULT_KS;
    public static volatile double BACK_KV = ShooterVelocityController.DEFAULT_KV;

    public static volatile double FRONT_TARGET_TPS = 0.0;
    public static volatile double FRONT_KP = ShooterVelocityController.DEFAULT_KP;
    public static volatile double FRONT_KS = ShooterVelocityController.DEFAULT_KS;
    public static volatile double FRONT_KV = ShooterVelocityController.DEFAULT_KV;

    private DcMotorEx back;
    private DcMotorEx front;
    private MultipleTelemetry telemetryM;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        try {
            back = hardwareMap.get(DcMotorEx.class, "back");
            front = hardwareMap.get(DcMotorEx.class, "front");
            configureMotor(back);
            configureMotor(front);

            dashboard = FtcDashboard.getInstance();
            telemetryM = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            telemetryM.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
            telemetryM.addLine("Dashboard: set BACK_TARGET_TPS and FRONT_TARGET_TPS to run each motor.");
            telemetryM.addData("Mapped motors", "back, front");
            telemetryM.update();

            waitForStart();
            long lastTelemetryNs = 0;
            while (opModeIsActive()) {
                double backVelocity = back.getVelocity();
                double frontVelocity = front.getVelocity();

                double backPower = ShooterVelocityController.calculate(
                        BACK_TARGET_TPS, backVelocity, BACK_KP, BACK_KS, BACK_KV);
                double frontPower = ShooterVelocityController.calculate(
                        FRONT_TARGET_TPS, frontVelocity, FRONT_KP, FRONT_KS, FRONT_KV);

                back.setPower(backPower);
                front.setPower(frontPower);

                long nowNs = System.nanoTime();
                if (nowNs - lastTelemetryNs >= TELEMETRY_INTERVAL_NS) {
                    lastTelemetryNs = nowNs;
                    telemetryM.addData("Back target (ticks/s)", BACK_TARGET_TPS);
                    telemetryM.addData("Back measured (ticks/s)", backVelocity);
                    telemetryM.addData("Back power", backPower);
                    telemetryM.addData("Front target (ticks/s)", FRONT_TARGET_TPS);
                    telemetryM.addData("Front measured (ticks/s)", frontVelocity);
                    telemetryM.addData("Front power", frontPower);
                    telemetryM.update();

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Back target (ticks/s)", BACK_TARGET_TPS);
                    packet.put("Back measured (ticks/s)", backVelocity);
                    packet.put("Front target (ticks/s)", FRONT_TARGET_TPS);
                    packet.put("Front measured (ticks/s)", frontVelocity);
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        } finally {
            if (back != null) back.setPower(0.0);
            if (front != null) front.setPower(0.0);
        }
    }

    private static void configureMotor(DcMotorEx motor) {
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
