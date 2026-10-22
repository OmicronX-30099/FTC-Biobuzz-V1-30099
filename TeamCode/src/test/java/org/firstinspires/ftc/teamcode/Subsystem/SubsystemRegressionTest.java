package org.firstinspires.ftc.teamcode.Subsystem;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders;
import org.firstinspires.ftc.teamcode.Util.CurrAlliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

import static org.junit.Assert.*;

public class SubsystemRegressionTest {
    private double savedGateOpenTPS;

    @Before
    public void useExplicitIntakeCalibration() {
        savedGateOpenTPS = Rollers.GATE_OPEN_TPS;
        Rollers.GATE_OPEN_TPS = 1100;
    }

    @After
    public void clearCommands() {
        Rollers.GATE_OPEN_TPS = savedGateOpenTPS;
        Scheduler.reset();
    }

    @Test
    public void loadingSubsystemsDoesNotReadBlocksHardwareMap() {
        assertFalse(Shooter.INSTANCE.getGateOpen());
        assertNotNull(Constants.drivetrainConfig);
        assertEquals(0.42, Flywheel.INSTANCE.calculate(1000, 1000, 0.003, 0.04, 0.00038), 1e-12);
    }

    @Test
    public void driveInitializesAndBrakesAllFourMotors() {
        TestHardware hardware = new TestHardware();
        MecanumDrive.INSTANCE.init(hardware);
        MecanumDrive.INSTANCE.setBrake(true);
        MecanumDrive.INSTANCE.drive(0.5, 0.0, 0.0);
        for (String name : new String[]{"flDrive", "frDrive", "blDrive", "brDrive"}) {
            Device motor = hardware.devices.get(name);
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.values.get("Mode"));
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, motor.values.get("ZeroPowerBehavior"));
            assertEquals(0.5, motor.number("Power"), 0.0);
        }
        assertEquals(DcMotorSimple.Direction.REVERSE, hardware.devices.get("flDrive").values.get("Direction"));
        assertEquals(DcMotorSimple.Direction.FORWARD, hardware.devices.get("frDrive").values.get("Direction"));
        MecanumDrive.INSTANCE.setBrake(false);
        MecanumDrive.INSTANCE.stop();
        for (String name : new String[]{"flDrive", "frDrive", "blDrive", "brDrive"}) {
            assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, hardware.devices.get(name).values.get("ZeroPowerBehavior"));
            assertEquals(0.0, hardware.devices.get(name).number("Power"), 0.0);
        }
    }

    @Test
    public void intakeCommandsRunInBothDirectionsAndResetBetweenRuns() {
        TestHardware hardware = new TestHardware();
        Rollers.INSTANCE.init(hardware);
        for (double power : new double[]{0.75, -0.5, 0.0, 0.75}) {
            Rollers.INSTANCE.changePow(power).schedule();
            Scheduler.execute();
            assertEquals(power, hardware.devices.get("left_intake").number("Power"), 0.0);
            assertEquals(power, hardware.devices.get("right_intake").number("Power"), 0.0);
        }
        Rollers.INSTANCE.stop();
        assertEquals(0.0, hardware.devices.get("left_intake").number("Power"), 0.0);
        TestHardware nextRun = new TestHardware();
        Rollers.INSTANCE.init(nextRun);
        Rollers.INSTANCE.changePow(0.75).schedule();
        Scheduler.execute();
        assertEquals(0.75, nextRun.devices.get("left_intake").number("Power"), 0.0);
    }

    @Test
    public void directIntakeControlSkipsDuplicatesAndStillStopsAtLowPower() {
        TestHardware hardware = new TestHardware();
        Rollers.INSTANCE.init(hardware);
        Device motor = hardware.devices.get("left_intake");
        Rollers.INSTANCE.setPower(0.75);
        int writes = motor.calls("setPower");
        Rollers.INSTANCE.setPower(0.75);
        assertEquals(writes, motor.calls("setPower"));
        Rollers.INSTANCE.setPower(-0.5);
        assertEquals(-0.5, motor.number("Power"), 0.0);
        Rollers.INSTANCE.setPower(0.015);
        Rollers.INSTANCE.setPower(0.004);
        assertEquals(0.004, motor.number("Power"), 0.0);
        Rollers.INSTANCE.setPower(0.0);
        assertEquals(0.0, motor.number("Power"), 0.0);
        assertEquals(0.0, hardware.devices.get("right_intake").number("Power"), 0.0);
    }

    @Test
    public void intakeLimitAppliesImmediatelyAndRestoresFullPower() {
        TestHardware hardware = new TestHardware();
        Rollers.INSTANCE.init(hardware);
        for (double direction : new double[]{1.0, -1.0}) {
            Rollers.INSTANCE.setPower(direction * 0.505);
            assertEquals(direction * 0.505, hardware.devices.get("left_intake").number("Power"), 0.0);
            Rollers.INSTANCE.setPower(direction * 0.505, 0.5);
            assertEquals(direction * 0.5, hardware.devices.get("left_intake").number("Power"), 0.0);
            assertEquals(direction * 0.5, hardware.devices.get("right_intake").number("Power"), 0.0);
            Rollers.INSTANCE.setPower(direction, 0.5);
            assertEquals(direction * 0.5, hardware.devices.get("left_intake").number("Power"), 0.0);
            Rollers.INSTANCE.setPower(direction * 0.4, 0.5);
            assertEquals(direction * 0.4, hardware.devices.get("left_intake").number("Power"), 0.0);
            Rollers.INSTANCE.setPower(direction, 1.0);
            assertEquals(direction, hardware.devices.get("left_intake").number("Power"), 0.0);
            Rollers.INSTANCE.setPower(0.0, 0.5);
            assertEquals(0.0, hardware.devices.get("left_intake").number("Power"), 0.0);
            assertEquals(0.0, hardware.devices.get("right_intake").number("Power"), 0.0);
        }
    }

    @Test
    public void intakeVelocityControlUsesLeftFeedbackAndPrioritizesTheOpenGate() {
        TestHardware hardware = new TestHardware();
        Rollers.INSTANCE.init(hardware);
        Device left = hardware.devices.get("left_intake");
        Device right = hardware.devices.get("right_intake");
        assertEquals(DcMotorSimple.Direction.FORWARD, left.values.get("Direction"));
        assertEquals(DcMotorSimple.Direction.REVERSE, right.values.get("Direction"));

        Rollers.INSTANCE.update(0.2f, 0.0f, false);
        assertEquals(0, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(0, left.number("Power"), 0);
        left.values.put("Velocity", 3000.0);
        right.values.put("Velocity", -5000.0); // Must not affect control.
        Rollers.INSTANCE.update(0.2001f, 0.0f, false);
        assertEquals(2800, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(0.89, left.number("Power"), 1e-12);
        assertEquals(left.number("Power"), right.number("Power"), 0);

        // A small feedback correction must not be swallowed by the manual power epsilon.
        left.values.put("Velocity", 3005.0);
        Rollers.INSTANCE.update(1.0f, 0.0f, false);
        assertEquals(0.887, left.number("Power"), 1e-12);
        assertEquals(left.number("Power"), right.number("Power"), 0);

        left.values.put("Velocity", 1100.0);
        Rollers.INSTANCE.update(1.0f, 0.0f, true);
        assertEquals(1100, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(0.415, left.number("Power"), 1e-12);
        Rollers.INSTANCE.update(0.0f, 0.0f, true);
        assertEquals(1100, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(0.415, right.number("Power"), 1e-12);
        Rollers.INSTANCE.update(0.0f, 0.0f, false);
        assertEquals(0, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(0, left.number("Power"), 0);
        assertEquals(0, right.number("Power"), 0);
        assertEquals(6, left.calls("getVelocity"));
        assertEquals(0, right.calls("getVelocity"));

        Rollers.INSTANCE.update(1.0f, 0.0f, false);
        Rollers.INSTANCE.stop();
        assertEquals(0, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(0, left.number("Power"), 0);
        assertEquals(0, right.number("Power"), 0);
    }

    @Test
    public void leftTriggerReversesBothIntakesWithGateOverride() {
        TestHardware hardware = new TestHardware();
        Rollers.INSTANCE.init(hardware);
        Device left = hardware.devices.get("left_intake");
        Device right = hardware.devices.get("right_intake");

        Rollers.INSTANCE.update(0.0f, 0.2f, false);
        assertEquals(0, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        left.values.put("Velocity", -2000.0);
        Rollers.INSTANCE.update(0.0f, 0.2001f, false);
        assertEquals(-2000, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertEquals(-0.73, left.number("Power"), 1e-12);
        assertEquals(left.number("Power"), right.number("Power"), 0);

        Rollers.INSTANCE.update(1.0f, 1.0f, false);
        assertEquals(-2000, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        Rollers.INSTANCE.update(1.0f, 1.0f, true);
        assertEquals(1100, Rollers.INSTANCE.getTargetVelocityTPS(), 0);
        assertTrue(left.number("Power") > 0);
        assertEquals(left.number("Power"), right.number("Power"), 0);

        Rollers.INSTANCE.update(0.0f, 0.0f, false);
        assertEquals(0, left.number("Power"), 0);
        assertEquals(0, right.number("Power"), 0);
        assertEquals(0, right.calls("getVelocity"));
        Rollers.INSTANCE.stop();
    }

    @Test
    public void driveSkipsDuplicateWritesWithoutLosingSmallInputChanges() {
        TestHardware hardware = new TestHardware();
        MecanumDrive.INSTANCE.init(hardware);
        MecanumDrive.INSTANCE.drive(0.5, 0.0, 0.0);
        Device motor = hardware.devices.get("flDrive");
        int writes = motor.calls("setPower");
        MecanumDrive.INSTANCE.drive(0.5, 0.0, 0.0);
        assertEquals(writes, motor.calls("setPower"));
        MecanumDrive.INSTANCE.drive(0.500001, 0.0, 0.0);
        assertEquals(writes + 1, motor.calls("setPower"));
        assertEquals(0.500001, motor.number("Power"), 0.0);
        MecanumDrive.INSTANCE.stop();
        MecanumDrive.INSTANCE.stop();
        assertEquals(writes + 3, motor.calls("setPower"));
        assertEquals(0.0, motor.number("Power"), 0.0);
        TestHardware nextRun = new TestHardware();
        MecanumDrive.INSTANCE.init(nextRun);
        MecanumDrive.INSTANCE.drive(0.500001, 0.0, 0.0);
        assertEquals(0.500001, nextRun.devices.get("flDrive").number("Power"), 0.0);
    }

    @Test
    public void shooterReadsVelocityOncePerUpdateAndKeepsFeedbackLive() {
        TestHardware hardware = new TestHardware();
        Shooter.INSTANCE.init(hardware);
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        try {
            CurrAlliance.INSTANCE.setCurrGoal(new Pose(0.0, 0.0));
            Device motor = hardware.devices.get("shooter");
            motor.values.put("Velocity", 985.48178);
            Shooter.INSTANCE.update(new Pose(0.0, 0.0));
            assertEquals(1, motor.calls("getVelocity"));
            assertEquals(0, motor.calls("getPower"));
            assertEquals(motor.number("Power"), Shooter.INSTANCE.getShooterPow(), 0.0);
            motor.values.put("Velocity", 0.0);
            Shooter.INSTANCE.update(new Pose(0.0, 0.0));
            assertEquals(2, motor.calls("getVelocity"));
            assertEquals(1.0, motor.number("Power"), 0.0);
            assertEquals(0.0, Shooter.INSTANCE.getShooterVelocity(), 0.0);
        } finally {
            Shooter.INSTANCE.stop();
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void turretUsesCorrectedRobotHeadingWithoutAdditionalAimTrim() {
        TestHardware hardware = new TestHardware();
        Turret.INSTANCE.init(hardware);
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        try {
            CurrAlliance.INSTANCE.setCurrGoal(new Pose(10, 0));
            Turret.INSTANCE.start();
            Turret.INSTANCE.update(new Pose(0, 0, Math.toRadians(2)));
            assertEquals(2, Turret.INSTANCE.getTargetDegrees(), 1e-9);
            Turret.INSTANCE.update(new Pose(0, 0, 0));
            assertEquals(0, Turret.INSTANCE.getTargetDegrees(), 1e-9);
        } finally {
            Turret.INSTANCE.stop();
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void shooterVelocityOffsetAddsToBaseShotClipsAtZeroAndCannotOverrideInvalidData() {
        boolean savedAuto = Flywheel.AUTO_DISTANCE_SHOT;
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        try {
            Flywheel.AUTO_DISTANCE_SHOT = true;
            CurrAlliance.INSTANCE.setCurrGoal(new Pose(0, 0));
            TestHardware hardware = new TestHardware();
            Shooter.INSTANCE.init(hardware);
            Pose pose = new Pose(36.12014673281381, 0);
            Shooter.INSTANCE.update(pose, 40);
            assertEquals(1480, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);
            Shooter.INSTANCE.update(pose, -20);
            assertEquals(1420, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);
            Shooter.INSTANCE.update(pose, -2000);
            assertEquals(0, Flywheel.INSTANCE.getFlyWheelVelo(), 0);
            Shooter.INSTANCE.update(new Pose(Double.NaN, 0), 2000);
            assertEquals(0, Flywheel.INSTANCE.getFlyWheelVelo(), 0);
            Shooter.INSTANCE.update(pose, 0);
            assertEquals(1440, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);
        } finally {
            Shooter.INSTANCE.stop();
            Flywheel.AUTO_DISTANCE_SHOT = savedAuto;
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void turretUsesLiveEncoderFeedbackAndStopsThroughShooterLifecycle() {
        TestHardware hardware = new TestHardware();
        Shooter.INSTANCE.init(hardware);
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        try {
            CurrAlliance.INSTANCE.setCurrGoal(new Pose(0.0, -10.0));
            Device motor = hardware.devices.get("turret");
            Device encoder = hardware.devices.get("frDrive");
            assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.values.get("Mode"));
            assertEquals(0, motor.calls("setTargetPosition"));
            assertEquals(0, encoder.calls("setMode"));
            assertEquals(0, encoder.calls("setPower"));
            // INIT never moves the turret, even if a subsystem update is called.
            Shooter.INSTANCE.update(new Pose(0, 0, 0));
            assertEquals(0, motor.number("Power"), 0);
            Shooter.INSTANCE.start();
            Shooter.INSTANCE.update(new Pose(0, 0, 0));
            assertEquals(90, Turret.INSTANCE.getTargetDegrees(), 1e-9);
            assertEquals(1.0, motor.number("Power"), 0.0);

            // Same target, new encoder reading: output must respond to motion.
            encoder.values.put("CurrentPosition", 428); // approximately 89.05 turret degrees
            encoder.values.put("Velocity", 384.5); // 80 turret degrees/s
            Shooter.INSTANCE.update(new Pose(0, 0, 0));
            assertTrue(motor.number("Power") < 0);
            encoder.values.put("Velocity", 0.0);
            Shooter.INSTANCE.update(new Pose(0, 0, 0));
            assertTrue(motor.number("Power") > 0);
            encoder.values.put("CurrentPosition", 432); // within 0.21 degrees of target
            Shooter.INSTANCE.update(new Pose(0, 0, 0));
            assertEquals(0, motor.number("Power"), 0);

            Shooter.INSTANCE.stop();
            assertEquals(0, motor.number("Power"), 0);
            Turret.INSTANCE.update(new Pose(0, 0, 0));
            assertEquals(0, motor.number("Power"), 0);
            TestHardware nextRun = new TestHardware();
            Shooter.INSTANCE.init(nextRun);
            assertEquals(0, Encoders.INSTANCE.getTrueDegrees(), 0);
            Shooter.INSTANCE.start();
            Shooter.INSTANCE.update(new Pose(0, 0, 0));
            assertEquals(1.0, nextRun.devices.get("turret").number("Power"), 0);
        } finally {
            Shooter.INSTANCE.stop();
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void turretReseedsAtStartAndBlocksOutwardPowerBeyondTravelLimits() {
        TestHardware hardware = new TestHardware();
        Turret.INSTANCE.init(hardware);
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        try {
            // Movement during INIT must be included in the starting absolute angle.
            hardware.absoluteVoltage = 3.3 * 45.0 / 360.0;
            Device encoder = hardware.devices.get("frDrive");
            encoder.values.put("CurrentPosition", 1000);
            Turret.INSTANCE.start();
            assertEquals(-10, Encoders.INSTANCE.getTrueDegrees(), 1e-9);

            // Seed at forward for the limit checks, without resetting the encoder input.
            hardware.absoluteVoltage = 0;
            encoder.values.put("CurrentPosition", 0);
            Turret.INSTANCE.start();
            CurrAlliance.INSTANCE.setCurrGoal(new Pose(10, 0));
            Device motor = hardware.devices.get("turret");
            encoder.values.put("CurrentPosition", -1154); // less than -230 degrees
            encoder.values.put("Velocity", 3845.0);
            Turret.INSTANCE.update(new Pose(0, 0, Math.toRadians(140)));
            assertTrue(Encoders.INSTANCE.getTrueDegrees() < -230);
            assertEquals(0, motor.number("Power"), 0);

            encoder.values.put("CurrentPosition", 1300); // more than +270 degrees
            encoder.values.put("Velocity", -3845.0);
            Turret.INSTANCE.update(new Pose(0, 0, Math.toRadians(-100)));
            assertTrue(Encoders.INSTANCE.getTrueDegrees() > 270);
            assertEquals(0, motor.number("Power"), 0);
            encoder.values.put("Velocity", 0.0);
            Turret.INSTANCE.update(new Pose(0, 0, Math.toRadians(-100)));
            assertTrue(motor.number("Power") < 0); // returning inside the range is allowed
        } finally {
            Turret.INSTANCE.stop();
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void flywheelSkipsIdenticalCommandsWithoutDroppingSmallPowerCorrections() {
        boolean savedAuto = Flywheel.AUTO_DISTANCE_SHOT;
        double savedTarget = Flywheel.TARGET_VELOCITY_TPS;
        double savedHood = Flywheel.HOOD_POSITION;
        TestHardware hardware = new TestHardware();
        try {
            Flywheel.AUTO_DISTANCE_SHOT = false;
            Flywheel.TARGET_VELOCITY_TPS = 1500;
            Flywheel.HOOD_POSITION = 0.35;
            Flywheel.INSTANCE.init(hardware);
            Device motor = hardware.devices.get("shooter");
            Device hood = hardware.devices.get("hood");
            Flywheel.INSTANCE.update(new Pose(0, 0), 1500);
            int motorWrites = motor.calls("setPower");
            int hoodWrites = hood.calls("setPosition");
            Flywheel.INSTANCE.update(new Pose(0, 0), 1500);
            assertEquals(motorWrites, motor.calls("setPower"));
            assertEquals(hoodWrites, hood.calls("setPosition"));

            Flywheel.HOOD_POSITION = 0.3505;
            Flywheel.INSTANCE.update(new Pose(0, 0), 1499.999);
            assertEquals(motorWrites + 1, motor.calls("setPower"));
            assertEquals(0.610003, motor.number("Power"), 1e-12);
            assertEquals(hoodWrites, hood.calls("setPosition"));
            Flywheel.HOOD_POSITION = 0.3512;
            Flywheel.INSTANCE.update(new Pose(0, 0), 1499.999);
            assertEquals(0.3512, hood.number("Position"), 1e-12);
            Flywheel.INSTANCE.stop();
            assertEquals(0, motor.number("Power"), 0);

            // Reinitializing must bind and command fresh hardware, regardless of cached values.
            TestHardware nextRun = new TestHardware();
            Flywheel.INSTANCE.init(nextRun);
            Flywheel.INSTANCE.update(new Pose(0, 0), 1500);
            assertEquals(0.61, nextRun.devices.get("shooter").number("Power"), 1e-12);
        } finally {
            Flywheel.INSTANCE.stop();
            Flywheel.AUTO_DISTANCE_SHOT = savedAuto;
            Flywheel.TARGET_VELOCITY_TPS = savedTarget;
            Flywheel.HOOD_POSITION = savedHood;
        }
    }

    @Test
    public void flywheelUsesMeasuredVelocityAndCommandsBoundedPower() {
        boolean savedAuto = Flywheel.AUTO_DISTANCE_SHOT;
        Flywheel.AUTO_DISTANCE_SHOT = false;
        TestHardware hardware = new TestHardware();
        Flywheel.INSTANCE.init(hardware);
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        try {
            CurrAlliance.INSTANCE.setCurrGoal(new Pose(0.0, 0.0));
            Device motor = hardware.devices.get("shooter");
            motor.values.put("Velocity", 1500.0);
            Flywheel.INSTANCE.update(new Pose(0.0, 0.0));
            assertEquals(1500.0, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);
            assertEquals(0.00038 * 1500.0 + 0.04, motor.number("Power"), 1e-9);
            motor.values.put("Velocity", 0.0);
            Flywheel.INSTANCE.update(new Pose(0.0, 0.0));
            assertEquals(1.0, motor.number("Power"), 0.0);
            motor.values.put("Velocity", 2000.0);
            Flywheel.INSTANCE.update(new Pose(0.0, 0.0));
            assertEquals(0.0, motor.number("Power"), 0.0);
            Flywheel.INSTANCE.stop();
            assertEquals(0.0, motor.number("Power"), 0.0);
        } finally {
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
            Flywheel.AUTO_DISTANCE_SHOT = savedAuto;
        }
    }

    @Test
    public void flywheelUsesLiveDashboardTargetWithoutResettingItAtInit() {
        boolean savedAuto = Flywheel.AUTO_DISTANCE_SHOT;
        Flywheel.AUTO_DISTANCE_SHOT = false;
        double savedTarget = Flywheel.TARGET_VELOCITY_TPS;
        TestHardware hardware = new TestHardware();
        try {
            Flywheel.TARGET_VELOCITY_TPS = 1800;
            Flywheel.INSTANCE.init(hardware);
            Device motor = hardware.devices.get("shooter");
            motor.values.put("Velocity", 1800.0);
            Flywheel.INSTANCE.update(new Pose(0, 0));
            assertEquals(1800, Flywheel.INSTANCE.getFlyWheelVelo(), 0);
            assertEquals(0.724, motor.number("Power"), 1e-9);

            Flywheel.TARGET_VELOCITY_TPS = 1200;
            motor.values.put("Velocity", 1200.0);
            Flywheel.INSTANCE.update(new Pose(0, 0));
            assertEquals(1200, Flywheel.INSTANCE.getFlyWheelVelo(), 0);
            assertEquals(0.496, motor.number("Power"), 1e-9);

            Flywheel.TARGET_VELOCITY_TPS = 0;
            Flywheel.INSTANCE.update(new Pose(0, 0));
            assertEquals(0, motor.number("Power"), 0);
        } finally {
            Flywheel.INSTANCE.stop();
            Flywheel.TARGET_VELOCITY_TPS = savedTarget;
            Flywheel.AUTO_DISTANCE_SHOT = savedAuto;
        }
    }

    @Test
    public void automaticShotTracksSelectedGoalAndSupportsManualDashboardOverride() {
        boolean savedAuto = Flywheel.AUTO_DISTANCE_SHOT;
        double savedTarget = Flywheel.TARGET_VELOCITY_TPS;
        double savedHood = Flywheel.HOOD_POSITION;
        org.firstinspires.ftc.teamcode.Util.Alliance savedAlliance = CurrAlliance.INSTANCE.getCur();
        boolean savedUseGoal1 = CurrAlliance.INSTANCE.getUseGoal1();
        Pose savedGoal = CurrAlliance.INSTANCE.getCurrGoal();
        TestHardware hardware = new TestHardware();
        try {
            Flywheel.AUTO_DISTANCE_SHOT = true;
            Flywheel.HOOD_POSITION = 0.35;
            Flywheel.INSTANCE.init(hardware);
            CurrAlliance.INSTANCE.setCur(new org.firstinspires.ftc.teamcode.Util.Alliance(0, 0, 100, 0));
            CurrAlliance.INSTANCE.SwitchGoal(1);
            Pose pose = new Pose(36.12014673281381, 0, 1.0);
            Flywheel.INSTANCE.update(pose, 1440);
            assertEquals(1440, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);
            assertEquals(0.485, hardware.devices.get("hood").number("Position"), 1e-12);
            Flywheel.INSTANCE.update(new Pose(pose.x(), pose.y(), -2.0), 1440);
            assertEquals(1440, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);

            CurrAlliance.INSTANCE.ToggleGoal();
            Flywheel.INSTANCE.update(pose, 1440);
            assertEquals(100 - pose.x(), Flywheel.INSTANCE.getDistanceToGoalInches(), 1e-9);
            assertTrue(Flywheel.INSTANCE.getFlyWheelVelo() > 1540);
            assertEquals(0.25, Flywheel.INSTANCE.getHoodPosition(), 1e-12);
            Flywheel.INSTANCE.update(new Pose(-200, 0), 1440);
            assertEquals(1600, Flywheel.INSTANCE.getFlyWheelVelo(), 1e-9);
            assertTrue(Flywheel.INSTANCE.getShotStatus().contains("outside measured range"));

            Flywheel.AUTO_DISTANCE_SHOT = false;
            Flywheel.TARGET_VELOCITY_TPS = 1700;
            Flywheel.HOOD_POSITION = 0.28;
            Flywheel.INSTANCE.update(pose, 1700);
            assertEquals(1700, Flywheel.INSTANCE.getFlyWheelVelo(), 0);
            assertEquals(0.28, hardware.devices.get("hood").number("Position"), 1e-12);
            Flywheel.AUTO_DISTANCE_SHOT = true;
            Flywheel.INSTANCE.update(new Pose(Double.NaN, 0), 1700);
            assertEquals(0, Flywheel.INSTANCE.getFlyWheelVelo(), 0);
            assertEquals(0, hardware.devices.get("shooter").number("Power"), 0);
            assertEquals(0.28, hardware.devices.get("hood").number("Position"), 1e-12);
        } finally {
            Flywheel.INSTANCE.stop();
            Flywheel.AUTO_DISTANCE_SHOT = savedAuto;
            Flywheel.TARGET_VELOCITY_TPS = savedTarget;
            Flywheel.HOOD_POSITION = savedHood;
            CurrAlliance.INSTANCE.setCur(savedAlliance);
            CurrAlliance.INSTANCE.SwitchGoal(savedUseGoal1 ? 1 : 2);
            CurrAlliance.INSTANCE.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void shooterRebindsHardwareAndClosesGateWhenStopped() {
        TestHardware hardware = new TestHardware();
        Shooter.INSTANCE.init(hardware);
        assertEquals(0.87, hardware.devices.get("gate").number("Position"), 0.0);
        assertEquals(0.35, hardware.devices.get("hood").number("Position"), 0.0);
        Shooter.INSTANCE.setGate(true);
        assertTrue(Shooter.INSTANCE.getGateOpen());
        assertEquals(0.75, hardware.devices.get("gate").number("Position"), 0.0);
        Shooter.INSTANCE.stop();
        assertFalse(Shooter.INSTANCE.getGateOpen());
        assertEquals(0.87, hardware.devices.get("gate").number("Position"), 0.0);
        assertEquals(0.0, hardware.devices.get("shooter").number("Power"), 0.0);

        TestHardware nextRun = new TestHardware();
        Shooter.INSTANCE.init(nextRun);
        Shooter.INSTANCE.setGate(true);
        assertEquals(0.75, nextRun.devices.get("gate").number("Position"), 0.0);
        assertEquals(0.87, hardware.devices.get("gate").number("Position"), 0.0);
        Shooter.INSTANCE.stop();
    }

    @Test
    public void shootingGateFollowsButtonStateWithoutRepeatedWrites() {
        TestHardware hardware = new TestHardware();
        Shooter.INSTANCE.init(hardware);
        try {
            Device gate = hardware.devices.get("gate");
            Shooter.INSTANCE.setGate(true);
            assertTrue(Shooter.INSTANCE.getGateOpen());
            assertEquals(0.75, gate.number("Position"), 0.0);
            int writes = gate.calls("setPosition");
            for (int i = 0; i < 100; i++) {
                Shooter.INSTANCE.setGate(true);
            }
            assertTrue(Shooter.INSTANCE.getGateOpen());
            assertEquals(0.75, gate.number("Position"), 0.0);
            assertEquals(writes, gate.calls("setPosition"));

            Shooter.INSTANCE.setGate(false);
            assertFalse(Shooter.INSTANCE.getGateOpen());
            assertEquals(0.87, gate.number("Position"), 0.0);
            Shooter.INSTANCE.setGate(false);
            assertEquals(writes + 1, gate.calls("setPosition"));
            Shooter.INSTANCE.setGate(true);
            assertEquals(0.75, gate.number("Position"), 0.0);
        } finally {
            Shooter.INSTANCE.stop();
        }
    }

    @Test
    public void flowersStartUpAndToggleOncePerBumperPress() {
        TestHardware hardware = new TestHardware();
        Flowers.INSTANCE.init(hardware);
        Device left = hardware.devices.get("left_flower");
        Device right = hardware.devices.get("right_flower");
        assertFalse(Flowers.INSTANCE.isDown());
        assertEquals(0.83, left.number("Position"), 0.0);
        assertTrue(right.callCounts.isEmpty());

        Flowers.INSTANCE.updateToggle(true);
        assertTrue(Flowers.INSTANCE.isDown());
        assertEquals(0.5, left.number("Position"), 0.0);
        int leftWrites = left.calls("setPosition");
        Flowers.INSTANCE.updateToggle(true);
        Flowers.INSTANCE.updateToggle(true);
        Flowers.INSTANCE.updateToggle(false);
        assertTrue(Flowers.INSTANCE.isDown());
        assertEquals(leftWrites, left.calls("setPosition"));
        assertTrue(right.callCounts.isEmpty());

        Flowers.INSTANCE.updateToggle(true);
        assertFalse(Flowers.INSTANCE.isDown());
        assertEquals(0.83, left.number("Position"), 0.0);
        Flowers.INSTANCE.updateToggle(true);
        assertFalse(Flowers.INSTANCE.isDown());
        assertTrue(right.callCounts.isEmpty());
    }

    @Test
    public void flowersReinitializeUpWithFreshButtonState() {
        TestHardware hardware = new TestHardware();
        hardware.devices.remove("right_flower");
        Flowers.INSTANCE.init(hardware);
        Flowers.INSTANCE.updateToggle(true);
        TestHardware nextRun = new TestHardware();
        nextRun.devices.remove("right_flower");
        Flowers.INSTANCE.init(nextRun);
        assertFalse(Flowers.INSTANCE.isDown());
        assertEquals(0.83, nextRun.devices.get("left_flower").number("Position"), 0.0);
        Flowers.INSTANCE.updateToggle(true);
        assertTrue(Flowers.INSTANCE.isDown());
        assertEquals(0.5, nextRun.devices.get("left_flower").number("Position"), 0.0);
    }

    // The SDK interfaces let these tests record hardware commands without Android or a robot.
    private static final class Device {
        final Map<String, Object> values = new HashMap<>();
        final Map<String, Integer> callCounts = new HashMap<>();
        final Object proxy;

        Device(Class<?> type) {
            if (type == DcMotorEx.class) {
                values.put("CurrentPosition", 0);
                values.put("Velocity", 0.0);
            }
            proxy = Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, (instance, method, args) -> {
                String name = method.getName();
                callCounts.merge(name, 1, Integer::sum);
                if (name.startsWith("set") && args.length == 1) {
                    values.put(name.substring(3), args[0]);
                    return null;
                }
                if (name.startsWith("get") && values.containsKey(name.substring(3))) {
                    return values.get(name.substring(3));
                }
                throw new UnsupportedOperationException(name);
            });
        }

        double number(String property) {
            return ((Number) values.get(property)).doubleValue();
        }

        int calls(String method) {
            return callCounts.getOrDefault(method, 0);
        }
    }

    private static final class TestHardware extends HardwareMap {
        final Map<String, Device> devices = new HashMap<>();
        double absoluteVoltage = 0;
        final AnalogInput absoluteEncoder = new AnalogInput(null, 0) {
            @Override public double getVoltage() { return absoluteVoltage; }
        };

        TestHardware() {
            super(null, null);
            for (String name : new String[]{"flDrive", "frDrive", "blDrive", "brDrive", "left_intake", "right_intake", "shooter", "turret"}) {
                devices.put(name, new Device(DcMotorEx.class));
            }
            devices.put("gate", new Device(Servo.class));
            devices.put("hood", new Device(Servo.class));
            devices.put("left_flower", new Device(Servo.class));
            devices.put("right_flower", new Device(Servo.class));
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            if (type == AnalogInput.class && name.equals("absEn")) return type.cast(absoluteEncoder);
            return type.cast(devices.get(name).proxy);
        }
    }
}
