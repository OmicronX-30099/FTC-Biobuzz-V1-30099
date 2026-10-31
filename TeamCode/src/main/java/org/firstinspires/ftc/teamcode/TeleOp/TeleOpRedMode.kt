package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.hardware.lynx.LynxModule
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import org.firstinspires.ftc.teamcode.Subsystem.Breakbeams
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.Subsystem.Flowers
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.Subsystem.Shooter
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.OpModeStorage
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.control.ShootingOffsets

@TeleOp(name = "TeleOpRed", group = "Red")
class TeleOpRedMode: LinearOpMode() {
    private lateinit var follower: Follower
    private val shootingOffsets = ShootingOffsets()
    private var lastLoopStartNanos = 0L
    private var lastTelemetryNanos = 0L
    private var circleWasPressed = false
    private var crossWasPressed = false
    private var squareWasPressed = false
    private var offsetsResetWasPressed = false
    private var revolutionLeftWasPressed = false
    private var revolutionRightWasPressed = false
    private lateinit var hubs: List<LynxModule>
    private lateinit var originalCacheModes: List<LynxModule.BulkCachingMode>
    private var lastControlMs = 0.0
    private var lastTelemetryMs = 0.0
    private val loopSamples = DoubleArray(100)
    private var loopSampleIndex = 0
    private var loopSampleCount = 0
    private var loopSampleSum = 0.0
    private var shooterInitialized = false
    private var rollersInitialized = false

    private companion object {
        const val TELEMETRY_INTERVAL_MS = 200
        const val TELEMETRY_INTERVAL_NANOS = TELEMETRY_INTERVAL_MS * 1_000_000L
    }

    override fun runOpMode() {
        try {
            initializeRobot()
            telemetry.addLine("Ready: TeleOpRed. Press START.")
            telemetry.update()
            waitForStart()
            if (isStopRequested) return
            startRobot()
            // Hardware reads provide blocking I/O; opModeIsActive yields and observes STOP.
            // Avoid the iterative OpMode's unconditional 1 ms sleep after every loop.
            while (opModeIsActive()) updateRobot()
        } finally {
            stopRobot()
        }
    }

    private fun initializeRobot() {
        telemetry.msTransmissionInterval = TELEMETRY_INTERVAL_MS
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        follower = Constants.create(hardwareMap)
        resetPoseToStart()
        CurrAlliance.Reset()
        Breakbeams.init(hardwareMap)
        hubs = hardwareMap.getAll(LynxModule::class.java)
        originalCacheModes = hubs.map { it.bulkCachingMode }
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    }

    private fun startRobot() {
        hubs.forEach { it.clearBulkCache() }
        OpModeStorage.autonomousEndPose?.let {
            follower.setPose(it)
            follower.update()
            OpModeStorage.autonomousEndPose = null
        }
        // These initializers command hardware positions, so they run only after START.
        Shooter.init(hardwareMap)
        shooterInitialized = true
        Rollers.init(hardwareMap)
        rollersInitialized = true
        Flowers.init(hardwareMap)
        Shooter.start()
        Shooter.toggleMode(Shooter.Mode.BACKUP)
        circleWasPressed = gamepad1.circle
        crossWasPressed = gamepad1.cross
        squareWasPressed = gamepad1.square
        offsetsResetWasPressed = gamepad1.triangle || gamepad2.triangle
        revolutionLeftWasPressed = gamepad2.circle
        revolutionRightWasPressed = gamepad2.square
        shootingOffsets.seedButtons(dpadLeftPressed(), dpadRightPressed(),
            dpadUpPressed(), dpadDownPressed())
        lastLoopStartNanos = 0L
        loopSampleCount = 0
        loopSampleIndex = 0
        loopSampleSum = 0.0
        lastTelemetryNanos = System.nanoTime() - TELEMETRY_INTERVAL_NANOS
    }

    private fun updateRobot() {
        val loopStartNanos = System.nanoTime()
        val previousLoopStartNanos = lastLoopStartNanos
        lastLoopStartNanos = loopStartNanos
        if (previousLoopStartNanos != 0L) {
            val sample = (loopStartNanos - previousLoopStartNanos) / 1_000_000.0
            if (loopSampleCount == loopSamples.size) loopSampleSum -= loopSamples[loopSampleIndex]
            else loopSampleCount++
            loopSamples[loopSampleIndex] = sample
            loopSampleSum += sample
            loopSampleIndex = (loopSampleIndex + 1) % loopSamples.size
        }
        // One hub snapshot serves velocity, encoder, analog and digital reads this loop.
        hubs.forEach { it.clearBulkCache() }
        if (Breakbeams.update(loopStartNanos)) {
            gamepad1.rumble(200)
            gamepad2.rumble(200)
        }

        val circlePressed = gamepad1.circle
        if (circlePressed && !circleWasPressed) resetPoseToStart()
        circleWasPressed = circlePressed

        val crossPressed = gamepad1.cross
        if (crossPressed && !crossWasPressed) {
            Shooter.toggleMode(if (Shooter.mode == Shooter.Mode.BACKUP)
                Shooter.Mode.NORMAL else Shooter.Mode.BACKUP)
        }
        crossWasPressed = crossPressed

        val squarePressed = gamepad1.square
        if (squarePressed && !squareWasPressed) Shooter.toggleSotm()
        squareWasPressed = squarePressed
        val offsetsResetPressed = gamepad1.triangle || gamepad2.triangle
        if (offsetsResetPressed && !offsetsResetWasPressed) {
            shootingOffsets.reset(dpadLeftPressed(), dpadRightPressed(),
                dpadUpPressed(), dpadDownPressed())
        }
        offsetsResetWasPressed = offsetsResetPressed
        val revolutionLeft = gamepad2.circle
        val revolutionRight = gamepad2.square
        if (!(revolutionLeft && revolutionRight)) {
            if (revolutionLeft && !revolutionLeftWasPressed) Encoders.shiftRevolution(-1)
            if (revolutionRight && !revolutionRightWasPressed) Encoders.shiftRevolution(1)
        }
        revolutionLeftWasPressed = revolutionLeft
        revolutionRightWasPressed = revolutionRight

        follower.manual(-gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(), -gamepad1.right_stick_x.toDouble())
        val driveStart = System.nanoTime()
        follower.update()
        val driveEnd = System.nanoTime()
        val pose = follower.pose()
        val fieldVelocity = follower.velocity()
        shootingOffsets.update(dpadLeftPressed(), dpadRightPressed(),
            dpadUpPressed(), dpadDownPressed())
        CurrAlliance.SelectGoalForY(pose.y())

        val shooting = gamepad1.left_bumper
        val intakeStart = System.nanoTime()
        Flowers.updateToggle(gamepad1.right_bumper)

        val shooterStart = System.nanoTime()
        Shooter.updateWithMotion(pose, fieldVelocity.vx, fieldVelocity.vy, fieldVelocity.omega,
            shootingOffsets.velocityTPS, shootingOffsets.aimOffsetDegrees)
        Shooter.setGate(shooting)
        Rollers.update(gamepad1.right_trigger, gamepad1.left_trigger, Shooter.gateOpen)
        val controlEnd = System.nanoTime()
        lastControlMs = (controlEnd - loopStartNanos) / 1_000_000.0

        // Status runs at 5 Hz; drive, localization and feedback remain at full loop rate.
        if (previousLoopStartNanos != 0L &&
            loopStartNanos - lastTelemetryNanos >= TELEMETRY_INTERVAL_NANOS) {
            val telemetryStart = System.nanoTime()
            val loopTimeMs = (loopStartNanos - previousLoopStartNanos) / 1_000_000.0
            telemetry.addData("Robot position (in)", "x: %.2f, y: %.2f", pose.x(), pose.y())
            telemetry.addData("Robot heading (deg)", "%.1f", Math.toDegrees(pose.heading()))
            telemetry.addData("Target goal", if (CurrAlliance.useGoal1) 1 else 2)
            telemetry.addData("Shooting mode (gamepad 1 Cross/A)", Shooter.mode)
            telemetry.addData("SOTM (gamepad 1 Square/X)", if (Shooter.sotmEnabled) "ON" else "OFF")
            telemetry.addData("Target flywheel velocity (ticks/s)", "%.1f", Flywheel.flyWheelVelo)
            telemetry.addData("Current flywheel velocity (ticks/s)", "%.1f", Shooter.shooterVelocity)
            telemetry.addData("Flywheel raw velocity (ticks/s)", "%.1f", Shooter.rawShooterVelocity)
            telemetry.addData("Hood position", "%.3f", Flywheel.hoodPosition)
            telemetry.addData("Distance to goal (in)", "%.2f", Flywheel.distanceToGoalInches)
            if (Shooter.mode == Shooter.Mode.NORMAL && Flywheel.AUTO_DISTANCE_SHOT) {
                telemetry.addData("Calibrated / commanded TPS", "%.1f / %.1f",
                    Flywheel.calibratedVelocityTPS, Flywheel.flyWheelVelo)
                telemetry.addData("Calibration endpoint held", Flywheel.shotClamped)
            }
            telemetry.addData("Turret angle / target (deg)", "%.2f / %.2f", Encoders.trueDegrees, Turret.targetDegrees)
            if (Shooter.mode == Shooter.Mode.BACKUP) {
                telemetry.addData("Encoder relative / absolute at check (deg)", "%.2f / %.2f",
                    Encoders.relativeDegreesAtCalibration, Encoders.absoluteDegreesAtCalibration)
                telemetry.addData("Encoder drift / largest (deg), correction accepted", "%.2f / %.2f / %s",
                    Encoders.calibrationErrorDegrees, Encoders.largestCalibrationErrorDegrees,
                    Encoders.calibrationAccepted)
            }
            telemetry.addData("Aim offset (deg)", "%.1f", shootingOffsets.aimOffsetDegrees)
            telemetry.addData("Shooter trim (ticks/s)", "%.0f", shootingOffsets.velocityTPS)
            // Start-to-start timing includes the previous loop's telemetry and SDK overhead.
            telemetry.addData("Loop time (ms)", "%.2f", loopTimeMs)
            var maximum = 0.0
            for (i in 0 until loopSampleCount) {
                maximum = maxOf(maximum, loopSamples[i])
            }
            telemetry.addData("Loop avg / max (last 100, ms)", "%.2f / %.2f",
                loopSampleSum / loopSampleCount, maximum)
            telemetry.addData("Control / drive / shooter (ms)", "%.2f / %.2f / %.2f",
                lastControlMs, (driveEnd - driveStart) / 1_000_000.0,
                (controlEnd - shooterStart) / 1_000_000.0)
            telemetry.addData("Inputs / intake+servos (ms)", "%.2f / %.2f",
                (driveStart - loopStartNanos) / 1_000_000.0,
                (shooterStart - intakeStart) / 1_000_000.0)
            telemetry.addData("Previous telemetry (ms)", "%.2f", lastTelemetryMs)
            telemetry.update()
            lastTelemetryMs = (System.nanoTime() - telemetryStart) / 1_000_000.0
            lastTelemetryNanos = loopStartNanos
        }
    }

    private fun resetPoseToStart() {
        shootingOffsets.reset(dpadLeftPressed(), dpadRightPressed(),
            dpadUpPressed(), dpadDownPressed())
        follower.setPose(Pose(8.2, 7.3, Math.toRadians(90.0)))
    }

    private fun dpadLeftPressed() = gamepad1.dpad_left || gamepad2.dpad_left
    private fun dpadRightPressed() = gamepad1.dpad_right || gamepad2.dpad_right
    private fun dpadUpPressed() = gamepad1.dpad_up || gamepad2.dpad_up
    private fun dpadDownPressed() = gamepad1.dpad_down || gamepad2.dpad_down

    private fun stopRobot() {
        try {
            if (shooterInitialized) Shooter.stop()
        } finally {
            try {
                if (rollersInitialized) Rollers.stop()
            } finally {
                try {
                    if (::follower.isInitialized) {
                        follower.manual(0.0, 0.0, 0.0)
                        follower.update()
                    }
                } finally {
                    if (::hubs.isInitialized && ::originalCacheModes.isInitialized) {
                        hubs.forEachIndexed { i, hub -> hub.bulkCachingMode = originalCacheModes[i] }
                    }
                }
            }
        }
    }
}
