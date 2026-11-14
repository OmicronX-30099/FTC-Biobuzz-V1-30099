package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Util.ServoPositions;

/** Live Dashboard control of one positional servo. */
@Config
@TeleOp(name = "Servo Test", group = "Tests")
public class ServoTest extends OpMode {
    public static volatile String SERVO_NAME = "left_flower";
    public static volatile double POSITION = ServoPositions.LEFT_FLOWER_UP - 0.05;
    public static volatile boolean REVERSED = false;

    private Servo servo;
    private String selectedName = "";
    private String connection = "";
    private double lastPosition = Double.NaN;
    private Servo.Direction lastDirection;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("Dashboard: ServoTest. Edit SERVO_NAME, POSITION (0–1), and REVERSED live.");
        selectServo();
        report();
    }

    @Override
    public void init_loop() {
        selectServo();
        report();
    }

    @Override
    public void loop() {
        selectServo();
        applyPosition();
        report();
    }

    private void selectServo() {
        String requestedName = SERVO_NAME == null ? "" : SERVO_NAME.trim();
        if (requestedName.equals(selectedName)) return;
        selectedName = requestedName;
        servo = selectedName.isEmpty() ? null : hardwareMap.tryGet(Servo.class, selectedName);
        connection = servo == null ? "" : servo.getConnectionInfo();
        lastPosition = Double.NaN;
        lastDirection = null;
    }

    private void applyPosition() {
        if (servo == null || !Double.isFinite(POSITION)) return;
        double position = Range.clip(POSITION, 0.0, 1.0);
        Servo.Direction direction = REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
        boolean directionChanged = direction != lastDirection;
        if (directionChanged) {
            servo.setDirection(direction);
            lastDirection = direction;
        }
        if (directionChanged || position != lastPosition) {
            if (Double.isNaN(lastPosition) && servo instanceof PwmControl) {
                ((PwmControl) servo).setPwmEnable();
            }
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    private void report() {
        telemetry.addData("Selected servo", selectedName.isEmpty() ? "Disabled" : selectedName);
        if (servo != null) {
            telemetry.addData("Connection", connection);
        } else if (!selectedName.isEmpty()) {
            telemetry.addLine("Servo not found. Check SERVO_NAME in Dashboard.");
        }
        telemetry.addData("Requested position", POSITION);
        telemetry.addData("Commanded position (0–1)", lastPosition);
        telemetry.addData("Direction", lastDirection);
        telemetry.addLine("Position is the last command, not physical servo feedback.");
        telemetry.update();
    }
}
