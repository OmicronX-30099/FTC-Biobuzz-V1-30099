package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystem.Rollers;

/** Exercises the same fixed intake powers and controls as TeleOp. */
@TeleOp(name = "Intake Test", group = "Tests")
public class IntakeTest extends OpMode {
    private MultipleTelemetry telemetryM;
    private boolean initialized;

    @Override
    public void init() {
        telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM.setMsTransmissionInterval(200);
        telemetryM.addLine("Press START. RT: intake 1.0; LT: reverse -0.3; LB: shooting feed 0.5.");
        telemetryM.update();
    }

    @Override
    public void start() {
        Rollers.INSTANCE.init(hardwareMap);
        initialized = true;
    }

    @Override
    public void loop() {
        Rollers.INSTANCE.update(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.left_bumper);
        telemetryM.addData("Intake power (both motors)", Rollers.INSTANCE.getIntakePower());
        telemetryM.update();
    }

    @Override
    public void stop() {
        if (initialized) Rollers.INSTANCE.stop();
    }
}
