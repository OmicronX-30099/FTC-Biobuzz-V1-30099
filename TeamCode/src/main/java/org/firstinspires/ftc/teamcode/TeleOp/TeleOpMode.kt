package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsystem.Flowers
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.Subsystem.Shooter
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.pedro.TeleConstants

@TeleOp(name = "TeleOpBlue", group = "Blue")
class TeleOpMode: OpMode() {
    private var lastLoopStartNanos = 0L
    private var lastTelemetryNanos = 0L

    private companion object {
        const val TELEMETRY_INTERVAL_MS = 100
        const val TELEMETRY_INTERVAL_NANOS = TELEMETRY_INTERVAL_MS * 1_000_000L
    }

    override fun init() {
        telemetry.msTransmissionInterval = TELEMETRY_INTERVAL_MS
        TeleConstants.init(hardwareMap)
        Shooter.init(hardwareMap)
        MecanumDrive.init(hardwareMap)
        MecanumDrive.setBrake(true)
        Rollers.init(hardwareMap)
        Flowers.init(hardwareMap)
    }

    override fun start() {
        lastLoopStartNanos = 0L
        lastTelemetryNanos = System.nanoTime() - TELEMETRY_INTERVAL_NANOS
    }

    override fun loop() {
        val loopStartNanos = System.nanoTime()
        val previousLoopStartNanos = lastLoopStartNanos
        lastLoopStartNanos = loopStartNanos

        MecanumDrive.drive( -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())

        val intakePow = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
        val shooting = gamepad1.left_bumper
        val maxIntakePower = if (shooting) 0.5 else 1.0
        Rollers.setPower(intakePow, maxIntakePower)

        Flowers.updateToggle(gamepad1.right_bumper)

        Shooter.setGate(shooting)
        Shooter.update()

        // Format and transmit at 10 Hz, using readings already collected by the control loop.
        if (previousLoopStartNanos != 0L &&
            loopStartNanos - lastTelemetryNanos >= TELEMETRY_INTERVAL_NANOS) {
            val pose = TeleConstants.follower.pose()
            val loopTimeMs = (loopStartNanos - previousLoopStartNanos) / 1_000_000.0
            telemetry.addData("Robot position (in)", "x: %.2f, y: %.2f", pose.x(), pose.y())
            telemetry.addData("Robot heading (deg)", "%.1f", Math.toDegrees(pose.heading()))
            telemetry.addData("Target flywheel velocity (ticks/s)", "%.1f", Flywheel.flyWheelVelo)
            telemetry.addData("Current flywheel velocity (ticks/s)", "%.1f", Shooter.shooterVelocity)
            telemetry.addData("Flowers", if (Flowers.isDown) "Down" else "Up")
            // Start-to-start timing includes the previous loop's telemetry and SDK overhead.
            telemetry.addData("Loop time (ms)", "%.2f", loopTimeMs)
            telemetry.update()
            lastTelemetryNanos = loopStartNanos
        }
    }

    override fun stop() {
        Shooter.stop()
        MecanumDrive.stop()
        Rollers.stop()
    }
}
