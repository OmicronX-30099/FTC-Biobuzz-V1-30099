package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystem.Rollers;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel;
import org.firstinspires.ftc.teamcode.control.ShooterRecoveryTimer;
import org.firstinspires.ftc.teamcode.control.DistanceShotModel;
import org.firstinspires.ftc.teamcode.Util.CurrAlliance;
import org.firstinspires.ftc.teamcode.Util.ServoPositions;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs feedback continuously and limits diagnostic telemetry, like TeleOp.
 * Uses TeleOp's Shooter and Rollers with distance velocity or a manual Dashboard target.
 */
@Config
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends LinearOpMode {
    private static final int TELEMETRY_INTERVAL_MS = 200;
    private static final long TELEMETRY_INTERVAL_NS = TELEMETRY_INTERVAL_MS * 1_000_000L;
    private static final Pose START_POSE = new Pose(8.2, 7.3, Math.toRadians(90.0));
    public static volatile double TARGET_VELOCITY_TPS = 1500.0;
    public static volatile boolean USE_DISTANCE_VELOCITY = true;
    public static volatile double HOOD_POSITION = ServoPositions.HOOD_DEFAULT;

    // Recovery is inferred from a dip after the shooter has settled at its target.
    public static volatile double RECOVERY_TOLERANCE_TPS = 20.0;
    public static volatile double SHOT_DROP_TPS = 50.0;
    public static volatile double RECOVERY_SETTLE_MS = 20.0;

    private DcMotorEx flywheel;
    private PinpointLocalizer localizer;
    private boolean turretTracking;
    private boolean turretToggleWasPressed;
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
            turretToggleWasPressed = gamepad1.a;
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
        Flywheel.KALMAN_FILTER_ENABLED = false;
        Flywheel.VOLTAGE_COMPENSATION_ENABLED = false;
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
        CurrAlliance.INSTANCE.Reset();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        localizer.setPose(START_POSE);
        Rollers.INSTANCE.init(hardwareMap);
        rollersInitialized = true;

        dashboard = FtcDashboard.getInstance();
        telemetryM = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetryM.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetryM.addData("Test version", "Shooter Test v15");
        telemetryM.addLine("Press START to command shooter.");
        telemetryM.addLine("Right trigger: intake; left trigger: reverse; hold left bumper to open gate and feed.");
        telemetryM.addLine("Dashboard: ShooterTest/Flywheel shot controls.");
        telemetryM.addLine("Distance polynomial enabled; disable USE_DISTANCE_VELOCITY for a manual TPS target.");
        telemetryM.addLine("Tune Flywheel KP (feedback), KS (static), KV (velocity feedforward) live.");
        telemetryM.addLine("Gains remain active across OpModes until app restart; save final values in code.");
        telemetryM.addLine("A toggles turret goal tracking. Place robot at (8.2, 7.3), heading 90 deg, before INIT.");
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
        localizer.update();
        Pose pose = localizer.pose();
        CurrAlliance.INSTANCE.SelectGoalForY(pose.y());
        Pose goal = CurrAlliance.INSTANCE.getCurrGoal();
        double distanceToGoal = Math.hypot(goal.x() - pose.x(), goal.y() - pose.y());

        boolean turretTogglePressed = gamepad1.a;
        if (turretTogglePressed && !turretToggleWasPressed) {
            turretTracking = !turretTracking;
            if (turretTracking) Shooter.INSTANCE.start();
            else Turret.INSTANCE.stop();
        }
        turretToggleWasPressed = turretTogglePressed;

        // Same control calls and order as TeleOpRed, using the selected test shot target.
        Shooter.INSTANCE.setAutoFlywheel(USE_DISTANCE_VELOCITY
                ? DistanceShotModel.velocityTPS(distanceToGoal) : TARGET_VELOCITY_TPS);
        Shooter.INSTANCE.setAutoHood(HOOD_POSITION);
        boolean shooting = gamepad1.left_bumper;
        Shooter.INSTANCE.update(pose);
        Shooter.INSTANCE.setGate(shooting);
        Rollers.INSTANCE.update(gamepad1.right_trigger, gamepad1.left_trigger,
                Shooter.INSTANCE.getGateOpen());

        double velocity = Shooter.INSTANCE.getRawShooterVelocity();
        double filteredVelocity = Shooter.INSTANCE.getShooterVelocity();
        double target = Flywheel.INSTANCE.getFlyWheelVelo();

        double nowSeconds = System.nanoTime() / 1e9;
        recoveryTimer.update(nowSeconds, target, velocity,
                RECOVERY_TOLERANCE_TPS, SHOT_DROP_TPS, RECOVERY_SETTLE_MS);

        // Diagnostics and current queries must not delay every velocity-control update.
        if (loopStartNs - lastTelemetryNs < TELEMETRY_INTERVAL_NS) return;
        lastTelemetryNs = loopStartNs;
        double rawVoltage = Flywheel.INSTANCE.getRawBatteryVoltage();
        double filteredVoltage = Flywheel.INSTANCE.getFilteredBatteryVoltage();
        double flywheelPower = Shooter.INSTANCE.getShooterPow();
        telemetryM.addData("Test version", "Shooter Test v15");
        telemetryM.addData("Turret tracking", turretTracking);
        telemetryM.addData("Turret angle / target (deg)", "%.2f / %.2f",
                Encoders.INSTANCE.getTrueDegrees(),
                Turret.INSTANCE.getTargetDegrees());
        telemetryM.addData("Robot pose", "%.2f, %.2f, %.1f deg", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()));
        telemetryM.addData("Distance to goal (in)", "%.2f", distanceToGoal);
        telemetryM.addData("Gate open", Shooter.INSTANCE.getGateOpen());
        telemetryM.addData("Intake power", Rollers.INSTANCE.getIntakePower());
        telemetryM.addData("Intake velocity target (ticks/s)", Rollers.INSTANCE.getIntakeVelocityTPS());
        telemetryM.addData("Motor mode", flywheel.getMode());
        telemetryM.addData("Current velocity (ticks/s)", velocity);
        telemetryM.addData("Filtered velocity (ticks/s)", filteredVelocity);
        telemetryM.addData("Velocity error (ticks/s)", target - filteredVelocity);
        telemetryM.addData("Target velocity (ticks/s)", target);
        telemetryM.addData("Flywheel KP / KS / KV", "%.6f / %.6f / %.8f",
                Flywheel.KP, Flywheel.KS, Flywheel.KV);
        telemetryM.addData("Battery raw / filtered (V)", "%.2f / %.2f",
                rawVoltage, filteredVoltage);
        telemetryM.addData("Flywheel motor power", flywheelPower);
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
        packet.put("Filtered velocity (ticks/s)", filteredVelocity);
        packet.put("Velocity error (ticks/s)", target - filteredVelocity);
        if (Double.isFinite(rawVoltage)) packet.put("Raw battery voltage (V)", rawVoltage);
        if (Double.isFinite(filteredVoltage)) packet.put("Filtered battery voltage (V)", filteredVoltage);
        packet.put("Flywheel motor power", flywheelPower);
        packet.put("Intake power", Rollers.INSTANCE.getIntakePower());
        packet.put("Intake velocity target (ticks/s)", Rollers.INSTANCE.getIntakeVelocityTPS());
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
