package org.firstinspires.ftc.teamcode.tests

import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController
import org.firstinspires.ftc.teamcode.experimental.physics.CalibrationLog
import org.firstinspires.ftc.teamcode.experimental.physics.CalibrationSettings
import org.firstinspires.ftc.teamcode.experimental.physics.PhysicsShooter
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.abs
import kotlin.math.hypot

/** Manual hood/TPS calibration with pose-based turret tracking; no physics LUT commands. */
@TeleOp(name = "Shooter Calibration Collector", group = "Experimental")
class ShooterCalibrationCollector : LinearOpMode() {
    override fun runOpMode() {
        val shooter = hardwareMap.get(DcMotorEx::class.java, "shooter")
        val hood = hardwareMap.get(Servo::class.java, "hood")
        val gate = hardwareMap.get(Servo::class.java, "gate")
        val left = hardwareMap.get(DcMotorEx::class.java, "left_intake")
        val right = hardwareMap.get(DcMotorEx::class.java, "right_intake")
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        val oldModes = hubs.map { it.bulkCachingMode }
        val settings = CalibrationSettings()
        val samples = CalibrationLog()
        val output = AppUtil.getInstance().getSettingsFile("shooter-calibration-${System.currentTimeMillis()}.csv")
        val startPose = Pose(8.2, 7.3, Math.toRadians(90.0))
        var started = false
        var turretInitialized = false
        try {
            for (motor in arrayOf(shooter, left, right)) {
                motor.power = 0.0
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            shooter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            left.direction = DcMotorSimple.Direction.FORWARD
            right.direction = DcMotorSimple.Direction.REVERSE
            hood.direction = Servo.Direction.FORWARD
            require(hood is PwmControl) { "Calibration hood requires programmable PWM range" }
            hood.pwmRange = PwmControl.PwmRange(PhysicsShooter.PWM_MIN_US, PhysicsShooter.PWM_MAX_US)
            gate.direction = Servo.Direction.FORWARD
            gate.position = ServoPositions.GATE_CLOSED
            hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
            val localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
            localizer.setPose(startPose)
            CurrAlliance.Reset()
            Turret.init(hardwareMap)
            turretInitialized = true
            telemetry.msTransmissionInterval = 200
            telemetry.addLine("MANUAL: 1480 TPS / hood .300. No LUT or peak-height restriction.")
            telemetry.addLine("D-pad Up/Down: +/-20 TPS. Right/Left: +/-0.005 hood servo.")
            telemetry.addLine("Cross: spin. RB: feed ONE ball per press; release after firing.")
            telemetry.addLine("After release: Square=SCORE, Triangle=MISS for latest attempt.")
            telemetry.addLine("LB: stop/reset settings, keep pose. Circle: also reset starting pose.")
            telemetry.addLine("Start at (8.2,7.3), 90 deg. Turret tracks the selected goal after START.")
            telemetry.addData("CSV written on STOP", output.absolutePath)
            telemetry.update()
            waitForStart()
            if (isStopRequested) return
            hubs.forEach { it.clearBulkCache() }
            Turret.start()
            started = true
            settings.seedButtons(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down)
            var wasCross = gamepad1.cross
            var wasSquare = gamepad1.square
            var wasTriangle = gamepad1.triangle
            var wasCircle = gamepad1.circle
            var wasLeftBumper = gamepad1.left_bumper
            var wasFeed = gamepad1.right_bumper
            var enabled = false
            var feedLatch = false
            var gateOpen = false
            var motorPower = 0.0
            var intakePower = 0.0
            var hoodCommand = Double.NaN
            var settledSince = 0L
            var previousTarget = Double.NaN
            var lastTelemetry = 0L
            val runStart = System.nanoTime()
            var message = "Ready; spin, wait for settled, feed one ball, release, then rate."
            while (opModeIsActive()) {
                val now = System.nanoTime()
                hubs.forEach { it.clearBulkCache() }
                val feedPressed = gamepad1.right_bumper
                val resetPose = gamepad1.circle && !wasCircle
                val resetSettings = resetPose || (gamepad1.left_bumper && !wasLeftBumper)
                if (resetSettings) {
                    enabled = false
                    feedLatch = false
                    settings.reset(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down)
                    if (resetPose) localizer.setPose(startPose)
                    message = "Stopped; baseline settings restored. Collected attempts retained."
                } else {
                    if (!feedPressed) settings.update(gamepad1.dpad_left, gamepad1.dpad_right,
                        gamepad1.dpad_up, gamepad1.dpad_down)
                    else settings.seedButtons(gamepad1.dpad_left, gamepad1.dpad_right,
                        gamepad1.dpad_up, gamepad1.dpad_down)
                    if (gamepad1.cross && !wasCross) enabled = !enabled
                }
                localizer.update()
                val pose = localizer.pose()
                CurrAlliance.SelectGoalForY(pose.y())
                // Reuse the measured pose; calibration shots are stationary, with no SOTM lead.
                Turret.update(pose)
                val goal = CurrAlliance.CurrGoal
                val distance = hypot(goal.x()-pose.x(), goal.y()-pose.y())
                val poseValid = distance.isFinite() && distance > 0 && pose.heading().isFinite()
                val target = if (enabled && poseValid) settings.targetTPS else 0.0
                val measured = shooter.velocity
                if (settings.hoodPosition != hoodCommand) {
                    hood.position = settings.hoodPosition
                    hoodCommand = settings.hoodPosition
                    settledSince = 0L
                }
                if (target != previousTarget || target <= 0 || !measured.isFinite() || abs(measured-target) > 20) {
                    settledSince = 0L
                } else if (settledSince == 0L) settledSince = now
                previousTarget = target
                val settled = settledSince != 0L && now-settledSince >= 150_000_000L
                if (!feedPressed || target <= 0 || !measured.isFinite()) feedLatch = false
                if (feedPressed && !wasFeed && target > 0 && measured.isFinite()) {
                    // Snapshot feedback BEFORE opening the gate; do not substitute speed sag later.
                    feedLatch = samples.capture((now-runStart)/1e9, pose.x(), pose.y(), pose.heading(),
                        if (CurrAlliance.useGoal1) 1 else 2, goal.x(), goal.y(), target, measured,
                        hoodCommand, settled)
                    message = if (feedLatch) "Attempt ${samples.count}: release RB, then Square=SCORE or Triangle=MISS."
                        else "Invalid sample; gate remains closed."
                }
                val feed = feedPressed && feedLatch && target > 0
                val power = ShooterVelocityController.calculate(target, measured,
                    ShooterVelocityController.DEFAULT_KP, ShooterVelocityController.DEFAULT_KS,
                    ShooterVelocityController.DEFAULT_KV)
                if (power != motorPower) { shooter.power = power; motorPower = power }
                if (feed != gateOpen) {
                    gate.position = if (feed) ServoPositions.GATE_OPEN else ServoPositions.GATE_CLOSED
                    gateOpen = feed
                }
                val transferPower = if (feed) Rollers.SHOOTING_POWER else 0.0
                if (transferPower != intakePower) {
                    left.power = transferPower; right.power = transferPower; intakePower = transferPower
                }
                val scorePressed = gamepad1.square && !wasSquare
                val missPressed = gamepad1.triangle && !wasTriangle
                if (!feed && scorePressed != missPressed && (scorePressed || missPressed)) {
                    message = if (samples.rateLast(scorePressed))
                        "Attempt ${samples.count} marked ${if (scorePressed) "SCORE" else "MISS"}."
                    else "No unrated latest attempt."
                }
                wasCross = gamepad1.cross
                wasSquare = gamepad1.square
                wasTriangle = gamepad1.triangle
                wasCircle = gamepad1.circle
                wasLeftBumper = gamepad1.left_bumper
                wasFeed = feedPressed
                if (now-lastTelemetry >= 200_000_000L) {
                    telemetry.addLine(message)
                    telemetry.addData("Distance / goal", "%.2f in / %d", distance, if (CurrAlliance.useGoal1) 1 else 2)
                    telemetry.addData("Robot XY / heading", "%.2f / %.2f / %.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()))
                    telemetry.addData("Motor TPS target / measured", "%.0f / %.1f", target, measured)
                    telemetry.addData("Configured TPS", settings.targetTPS)
                    telemetry.addData("Turret angle / target (deg)", "%.2f / %.2f",
                        Encoders.trueDegrees, Turret.targetDegrees)
                    telemetry.addData("Hood servo / degrees", "%.4f / %.2f", hoodCommand,
                        PhysicsShooter.hoodDegreesFromServoPosition(hoodCommand))
                    telemetry.addData("Settled (within 20 TPS for 150 ms)", settled)
                    telemetry.addData("Attempts / scores / misses / unrated", "%d / %d / %d / %d",
                        samples.count, samples.scores, samples.misses, samples.unrated)
                    telemetry.addLine("Cross=spin; D-pad U/D=TPS, L/R=hood; RB=one feed; Square=score; Triangle=miss")
                    telemetry.addLine("LB=stop/reset settings; Circle=also reset pose. CSV saved on STOP.")
                    telemetry.addData("CSV", output.absolutePath)
                    telemetry.update()
                    lastTelemetry = now
                }
            }
        } finally {
            try {
                try { if (turretInitialized) Turret.stop() } finally {
                    try { shooter.power = 0.0 } finally {
                        try { left.power = 0.0 } finally {
                            try { right.power = 0.0 } finally { gate.position = ServoPositions.GATE_CLOSED }
                        }
                    }
                }
            } finally {
                try {
                    hubs.forEachIndexed { i, hub -> hub.bulkCachingMode = oldModes[i] }
                } finally {
                    if (started) {
                        try {
                            output.writeText(samples.toCsv(), Charsets.UTF_8)
                            telemetry.addData("Saved calibration CSV", output.absolutePath)
                            telemetry.addData("Attempts", samples.count)
                        } catch (error: Exception) {
                            telemetry.addData("CSV save failed", error.message)
                        }
                        telemetry.update()
                    }
                }
            }
        }
    }
}
