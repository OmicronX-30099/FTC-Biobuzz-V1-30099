package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Select up to two positional servos during INIT, then command them independently after START. */
@TeleOp(name = "Servo Test", group = "Tests")
public class ServoTest extends OpMode {
    public static volatile String SERVO_NAME = "hood";
    public static volatile double POSITION = 0.16;
    public static volatile boolean REVERSED = false;

    // Leave the second name blank to test only the first servo.
    public static volatile String SERVO_NAME_2 = "";
    public static volatile double POSITION_2 = 0.5;
    public static volatile boolean REVERSED_2 = false;

    private final ServoSlot first = new ServoSlot("Servo 1");
    private final ServoSlot second = new ServoSlot("Servo 2");

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);
        selectServos();
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        selectServos();
        showInitTelemetry();
    }

    private void selectServos() {
        first.select(SERVO_NAME);
        second.select(SERVO_NAME_2);
    }

    private boolean sameServoSelected() {
        return first.servo != null && first.servo == second.servo;
    }

    private void showInitTelemetry() {
        telemetry.addLine("Configure names, positions and reversal flags in ServoTest.java before building.");
        telemetry.addLine("Leave a name blank to disable that slot.");
        telemetry.addLine("Press START to apply the source-defined positions and reversal flags.");
        showSelectionTelemetry();
        telemetry.addData("Servo 1 requested position", POSITION);
        telemetry.addData("Servo 2 requested position", POSITION_2);
        telemetry.update();
    }

    @Override
    public void loop() {
        double firstPosition = POSITION;
        double secondPosition = POSITION_2;
        first.apply(firstPosition, REVERSED);
        if (!sameServoSelected()) {
            second.apply(secondPosition, REVERSED_2);
        }

        showSelectionTelemetry();
        first.showCommandTelemetry(firstPosition);
        second.showCommandTelemetry(secondPosition);
        telemetry.addLine("Position is the last command, not physical servo feedback.");
        telemetry.addLine("To switch servo names, stop and press INIT again.");
        telemetry.update();
    }

    private void showSelectionTelemetry() {
        first.showSelection();
        second.showSelection();
        if (sameServoSelected()) {
            telemetry.addLine("Both slots select the same servo. Only Servo 1 will command it; choose different names during INIT.");
        }
    }

    private final class ServoSlot {
        private final String label;
        private Servo servo;
        private String selectedName;
        private String connection = "";
        private double lastPosition = Double.NaN;
        private Servo.Direction lastDirection;

        private ServoSlot(String label) {
            this.label = label;
        }

        private void select(String requestedName) {
            requestedName = requestedName == null ? "" : requestedName.trim();
            if (requestedName.equals(selectedName)) {
                return;
            }
            selectedName = requestedName;
            servo = requestedName.isEmpty() ? null : hardwareMap.tryGet(Servo.class, requestedName);
            connection = servo == null ? "" : servo.getConnectionInfo();
            lastPosition = Double.NaN;
            lastDirection = null;
        }

        private void apply(double requestedPosition, boolean reversed) {
            if (servo == null || !Double.isFinite(requestedPosition)) {
                return;
            }
            double position = Range.clip(requestedPosition, 0.0, 1.0);
            Servo.Direction direction = reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
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

        private void showSelection() {
            telemetry.addData(label + " selected name", selectedName.isEmpty() ? "Disabled" : selectedName);
            if (servo != null) {
                telemetry.addData(label + " connection", connection);
            } else if (!selectedName.isEmpty()) {
                telemetry.addLine(label + ": no positional servo found. Check its configuration name during INIT.");
            }
        }

        private void showCommandTelemetry(double requestedPosition) {
            telemetry.addData(label + " requested position", requestedPosition);
            telemetry.addData(label + " commanded position (0 to 1)", lastPosition);
            telemetry.addData(label + " direction", lastDirection);
            if (!Double.isFinite(requestedPosition)) {
                telemetry.addLine(label + ": invalid position. Enter a finite number; keeping the previous command.");
            }
        }
    }
}
