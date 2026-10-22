package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Subsystem.Breakbeams;

import java.util.List;

/** Raw sensor status only: no motors, servos, debounce, or ball counting. */
@TeleOp(name = "Beam Break Test", group = "Tests")
public class BreakbeamTest extends OpMode {
    private String[] names;
    private DigitalChannel[] beams;
    private List<LynxModule> hubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        names = new String[]{Breakbeams.BEAM_1_NAME, Breakbeams.BEAM_2_NAME,
                Breakbeams.BEAM_3_NAME, Breakbeams.BEAM_4_NAME};
        beams = new DigitalChannel[names.length];
        for (int i = 0; i < names.length; i++) {
            beams[i] = hardwareMap.get(DigitalChannel.class, names[i]);
            beams[i].setMode(DigitalChannel.Mode.INPUT);
        }
        hubs = hardwareMap.getAll(LynxModule.class);
        showStatus();
    }

    @Override
    public void init_loop() {
        showStatus();
    }

    @Override
    public void loop() {
        showStatus();
    }

    private void showStatus() {
        // Keep readings fresh even if a previous OpMode left MANUAL bulk caching enabled.
        for (LynxModule hub : hubs) hub.clearBulkCache();
        boolean blockedWhenLow = Breakbeams.BLOCKED_WHEN_LOW;
        for (int i = 0; i < beams.length; i++) {
            boolean high = beams[i].getState();
            boolean blocked = blockedWhenLow ? !high : high;
            telemetry.addData(names[i], "%s (%s)", blocked ? "BLOCKED" : "CLEAR", high ? "HIGH" : "LOW");
        }
        telemetry.update();
    }
}
