package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystem.Rollers;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel;
import org.firstinspires.ftc.teamcode.control.ShooterRecoveryTimer;
import org.firstinspires.ftc.teamcode.Util.ServoPositions;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs feedback continuously and limits diagnostic telemetry, like TeleOp.
 * Uses TeleOp's Shooter and Rollers subsystems with manual Dashboard shot targets.
 */
@Config
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends LinearOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 200;
    private static final long TELEMETRY_INTERVAL_NS = TELEMETRY_INTERVAL_MS * 1_000_000L;
    private static final Pose TEST_POSE = new Pose(0, 0, 0);
    public static volatile double TARGET_VELOCITY_TPS = 1500.0;
    public static volatile double HOOD_POSITION = ServoPositions.HOOD_DEFAULT;

    // Recovery is inferred from a dip after the shooter has settled at its target.
    public static volatile double RECOVERY_TOLERANCE_TPS = 20.0;
    public static volatile double SHOT_DROP_TPS = 50.0;
    public static volatile double RECOVERY_SETTLE_MS = 20.0;

    private DcMotorEx flywheel;
    private boolean shooterInitialized;
    private boolean rollersInitialized;
    private List<LynxModule> hubs;
    private final List<LynxModule.BulkCachingMode> originalCacheModes = new ArrayList<>();
    private MultipleTelemetry telemetryM;
    private FtcDashboard dashboard;
    private final ShooterRecoveryTimer recoveryTimer = new ShooterRecoveryTimer();
    private long lastTelemetryNs;
    private long lastLoopStartNs;
    private double maximumLoopIntervalMs;

    @Override
    public void runOpMode() {
        try {
            initializeRobot();
            waitForStart();
            if (isStopRequested()) return;
            lastTelemetryNs = System.nanoTime() - TELEMETRY_INTERVAL_NS;
            lastLoopStartNs = 0;
            maximumLoopIntervalMs = 0;
            while (opModeIsActive()) updateRobot();
        } finally {
            stopRobot();
        }
    }

    private void initializeRobot() {
        recoveryTimer.reset();
        hubs = hardwareMap.getAll(LynxModule.class);
        originalCacheModes.clear();
        for (LynxModule hub : hubs) {
            originalCacheModes.add(hub.getBulkCachingMode());
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }
        Shooter.INSTANCE.init(hardwareMap);
        shooterInitialized = true;
        Shooter.INSTANCE.toggleMode(Shooter.Mode.AUTO);
        flywheel = Shooter.INSTANCE.getShooter();
        Rollers.INSTANCE.init(hardwareMap);
        rollersInitialized = true;

        dashboard = FtcDashboard.getInstance();
        telemetryM = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetryM.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetryM.addData("Test version", "Shooter Test v13");
        telemetryM.addLine("Press START to command shooter.");
        telemetryM.addLine("Right trigger: intake; left trigger: reverse; left bumper: open gate and feed.");
        telemetryM.addLine("Tune target velocity and hood from Dashboard; controller gains match TeleOp.");
        telemetryM.addLine("Turret stays inactive during this stationary shooter test.");
        telemetryM.addData("Motor connection", flywheel.getConnectionInfo());
        telemetryM.update();
    }

    private void updateRobot() {
        long loopStartNs = System.nanoTime();
        double loopIntervalMs = lastLoopStartNs == 0 ? 0 : (loopStartNs - lastLoopStartNs) / 1e6;
        lastLoopStartNs = loopStartNs;
        maximumLoopIntervalMs = Math.max(maximumLoopIntervalMs, loopIntervalMs);
        // Match TeleOp: both velocity controllers must see fresh feedback every loop.
        for (LynxModule hub : hubs) hub.clearBulkCache();

        // Same control calls and order as TeleOpRed, using manual test shot targets.
        Shooter.INSTANCE.setAutoFlywheel(TARGET_VELOCITY_TPS);
        Shooter.INSTANCE.setAutoHood(HOOD_POSITION);
        boolean shooting = gamepad1.left_bumper;
        // Shooter.start() activates turret aiming; leave it inactive for this test.
        Shooter.INSTANCE.update(TEST_POSE);
        Shooter.INSTANCE.setGate(shooting);
        Rollers.INSTANCE.update(gamepad1.right_trigger, gamepad1.left_trigger,
                Shooter.INSTANCE.getGateOpen());

        double velocity = Shooter.INSTANCE.getRawShooterVelocity();
        double target = Flywheel.INSTANCE.getFlyWheelVelo();

        double nowSeconds = System.nanoTime() / 1e9;
        recoveryTimer.update(nowSeconds, target, velocity,
                RECOVERY_TOLERANCE_TPS, SHOT_DROP_TPS, RECOVERY_SETTLE_MS);

        // Diagnostics and current queries must not delay every velocity-control update.
        if (loopStartNs - lastTelemetryNs < TELEMETRY_INTERVAL_NS) return;
        lastTelemetryNs = loopStartNs;
        telemetryM.addData("Test version", "Shooter Test v11");
        telemetryM.addData("Gate open", Shooter.INSTANCE.getGateOpen());
        telemetryM.addData("Intake power", Rollers.INSTANCE.getIntakePower());
        telemetryM.addData("Motor mode", flywheel.getMode());
        telemetryM.addData("Current velocity (ticks/s)", velocity);
        telemetryM.addData("Target velocity (ticks/s)", target);
        telemetryM.addData("Control interval (ms)", loopIntervalMs);
        telemetryM.addData("Max control interval since last telemetry (ms)", maximumLoopIntervalMs);
        maximumLoopIntervalMs = 0;
        telemetryM.addData("Recovery status", recoveryTimer.getState());
        telemetryM.addData("Recovery elapsed (ms)", recoveryTimer.getElapsedMs(nowSeconds));
        telemetryM.addData("Last recovery (ms)", recoveryTimer.getLastRecoveryMs());
        telemetryM.addData("Last velocity drop (ticks/s)", recoveryTimer.getLastDropTPS());
        telemetryM.addData("Completed recoveries", recoveryTimer.getCompletedRecoveries());
        if (recoveryTimer.getState() == ShooterRecoveryTimer.State.INVALID_INPUT) {
            telemetryM.addLine("Recovery settings: use finite values, drop > tolerance >= 0, settle >= 0.");
        }
        telemetryM.addData("Motor current (A)", flywheel.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Motor hardware enabled", flywheel.isMotorEnabled());
        telemetryM.update();

        // Keep fixed series names so Dashboard can graph target and measured velocity over time.
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target velocity (ticks/s)", target);
        packet.put("Actual velocity (ticks/s)", velocity);
        packet.put("Intake power", Rollers.INSTANCE.getIntakePower());
        dashboard.sendTelemetryPacket(packet);
    }

    private void stopRobot() {
        try {
            if (shooterInitialized) {
                Shooter.INSTANCE.stop();
            }
        } finally {
            try {
                if (rollersInitialized) {
                    Rollers.INSTANCE.stop();
                }
            } finally {
                for (int i = 0; i < originalCacheModes.size(); i++) {
                    hubs.get(i).setBulkCachingMode(originalCacheModes.get(i));
                }
            }
        }
    }
}
