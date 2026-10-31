package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.PwmControl
import com.pedropathing.math.Pose
import com.pedropathing.revhub.localizers.PinpointLocalizer
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController
import org.firstinspires.ftc.teamcode.experimental.physics.PhysicsShooter
import org.firstinspires.ftc.teamcode.experimental.physics.PhysicsShot
import org.firstinspires.ftc.teamcode.experimental.physics.PhysicsVelocityTrim
import kotlin.math.abs
import kotlin.math.hypot

/** Pose-based stationary experiment. Pedro localization only; no drivetrain or turret commands. */
@TeleOp(name = "Physics Shooter Test", group = "Experimental")
class PhysicsShooterTest : LinearOpMode() {
    override fun runOpMode() {
        val shooter = hardwareMap.get(DcMotorEx::class.java, "shooter")
        val hood = hardwareMap.get(Servo::class.java, "hood")
        val gate = hardwareMap.get(Servo::class.java, "gate")
        val left = hardwareMap.get(DcMotorEx::class.java, "left_intake")
        val right = hardwareMap.get(DcMotorEx::class.java, "right_intake")
        val hubs = hardwareMap.getAll(LynxModule::class.java)
        val oldModes = hubs.map { it.bulkCachingMode }
        val localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        val startPose = Pose(8.2, 7.3, Math.toRadians(90.0))
        var enabled = false
        var allowFallback = false
        val shot = PhysicsShot()
        val velocityTrim = PhysicsVelocityTrim()
        var lastShooterPower = 0.0
        var lastIntakePower = 0.0
        var lastHood = Double.NaN
        var gateOpen = false
        try {
            for (motor in arrayOf(shooter, left, right)) {
                motor.power = 0.0
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
            shooter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            left.direction = DcMotorSimple.Direction.FORWARD
            right.direction = DcMotorSimple.Direction.REVERSE
            hood.direction = Servo.Direction.FORWARD
            require(hood is PwmControl) { "Physics hood requires programmable PWM range" }
            // The supplied 90%-travel geometry requires 600–2400 us, not 500–2500 us.
            hood.pwmRange = PwmControl.PwmRange(PhysicsShooter.PWM_MIN_US, PhysicsShooter.PWM_MAX_US)
            gate.direction = Servo.Direction.FORWARD
            gate.position = ServoPositions.GATE_CLOSED
            hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
            CurrAlliance.Reset()
            localizer.setPose(startPose)
            telemetry.msTransmissionInterval = 200
            telemetry.addLine("EXPERIMENT: drag coefficient assumed; stationary shots only.")
            telemetry.addLine("Hood PWM: 600–2400 us; 355-degree servo, 30/148 hood ratio.")
            telemetry.addLine("Distance: Pedro pose to active goal. Goal 2 above Y=70.75 in.")
            telemetry.addLine("Starting pose: (8.2, 7.3) in, heading 90 deg. Align turret manually.")
            telemetry.addLine("Cross: spin on/off. Triangle: allow height-infeasible fallback.")
            telemetry.addLine("D-pad L/R: multiplier -/+0.01. Down/Up: offset -/+20 TPS.")
            telemetry.addLine("Hold right bumper: feed. Circle: stop/reset pose and velocity trim.")
            telemetry.update()
            waitForStart()
            if (isStopRequested) return
            velocityTrim.seedButtons(gamepad1.dpad_left, gamepad1.dpad_right,
                gamepad1.dpad_up, gamepad1.dpad_down)
            var wasCross = gamepad1.cross
            var wasTriangle = gamepad1.triangle
            var wasCircle = gamepad1.circle
            var lastTelemetry = 0L
            var lastLoop = 0L
            while (opModeIsActive()) {
                val now = System.nanoTime()
                hubs.forEach { it.clearBulkCache() }
                val reset = gamepad1.circle && !wasCircle
                if (reset) {
                    enabled = false
                    allowFallback = false
                    localizer.setPose(startPose)
                    velocityTrim.reset(gamepad1.dpad_left, gamepad1.dpad_right,
                        gamepad1.dpad_up, gamepad1.dpad_down)
                } else {
                    velocityTrim.update(gamepad1.dpad_left, gamepad1.dpad_right,
                        gamepad1.dpad_up, gamepad1.dpad_down)
                    if (gamepad1.cross && !wasCross) enabled = !enabled
                    if (gamepad1.triangle && !wasTriangle) allowFallback = !allowFallback
                }
                wasCross = gamepad1.cross
                wasTriangle = gamepad1.triangle
                wasCircle = gamepad1.circle
                // Exactly one Pinpoint update per loop, using the production Pedro pod configuration.
                localizer.update()
                val pose = localizer.pose()
                CurrAlliance.SelectGoalForY(pose.y())
                val goal = CurrAlliance.CurrGoal
                val distance = hypot(goal.x()-pose.x(), goal.y()-pose.y())
                // Preserve real distance: out-of-range/invalid pose must stop shooting, not clamp.
                PhysicsShooter.calculateShot(distance, shot)
                val adjustedTPS = if (shot.valid) velocityTrim.apply(shot.shooterVelocityTPS) else 0.0
                val canShoot = enabled && shot.valid && (shot.feasible || allowFallback) && adjustedTPS > 0.0
                val target = if (canShoot) adjustedTPS else 0.0
                val measured = shooter.velocity
                val power = ShooterVelocityController.calculate(target, measured,
                    ShooterVelocityController.DEFAULT_KP, ShooterVelocityController.DEFAULT_KS,
                    ShooterVelocityController.DEFAULT_KV)
                if (power != lastShooterPower) {
                    shooter.power = power
                    lastShooterPower = power
                }
                val hoodPosition = PhysicsShooter.hoodServoPosition(shot.hoodAngleDegrees)
                if (shot.valid && (lastHood.isNaN() || abs(hoodPosition-lastHood) >= 0.001)) {
                    hood.position = hoodPosition
                    lastHood = hoodPosition
                }
                val feed = canShoot && gamepad1.right_bumper
                if (feed != gateOpen) {
                    gate.position = if (feed) ServoPositions.GATE_OPEN else ServoPositions.GATE_CLOSED
                    gateOpen = feed
                }
                val intakePower = if (feed) Rollers.SHOOTING_POWER else 0.0
                if (intakePower != lastIntakePower) {
                    left.power = intakePower
                    right.power = intakePower
                    lastIntakePower = intakePower
                }
                if (now-lastTelemetry >= 200_000_000L) {
                    telemetry.addLine("PROVISIONAL aerodynamic model; no SOTM. Read doc/PhysicsShooter.md.")
                    telemetry.addData("Distance (in)", distance)
                    telemetry.addData("Target height above shooter (cm)", PhysicsShooter.TARGET_HEIGHT_CM)
                    telemetry.addData("Robot XY (in) / heading (deg)", "%.2f / %.2f / %.1f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()))
                    telemetry.addData("Goal / XY (in)", "%d / %.2f / %.2f",
                        if (CurrAlliance.useGoal1) 1 else 2, goal.x(), goal.y())
                    telemetry.addData("LUT status (before trim)", shot.status)
                    telemetry.addData("Enabled / fallback allowed", "$enabled / $allowFallback")
                    telemetry.addData("TPS target / measured", "%.1f / %.1f", target, measured)
                    telemetry.addData("Velocity m / b (TPS)", "%.2f / %.0f", velocityTrim.multiplier, velocityTrim.offsetTPS)
                    telemetry.addData("LUT TPS / adjusted TPS", "%.1f / %.1f", shot.shooterVelocityTPS, adjustedTPS)
                    telemetry.addData("Predicted TPS / hood deg", "%.1f / %.1f", shot.shooterVelocityTPS, shot.hoodAngleDegrees)
                    telemetry.addData("Hood servo command", lastHood)
                    telemetry.addData("LUT launch / entry down (deg)", "%.1f / %.1f", shot.launchAngleDegrees, shot.entryAngleDegrees)
                    telemetry.addData("LUT preferred entry / error (deg)", "%.1f / %.1f",
                        PhysicsShooter.PREFERRED_ENTRY_DEGREES, shot.entryAngleErrorDegrees)
                    telemetry.addData("Vertex height cm / distance in", "%.2f / %.2f", shot.vertexHeightCm, shot.vertexDistanceInches)
                    telemetry.addData("Height >=136cm / feasible", "${shot.heightConstraintSatisfied} / ${shot.feasible}")
                    telemetry.addData("Flight time (s)", "%.3f", shot.flightTimeSeconds)
                    telemetry.addData("Loop (ms)", "%.2f", if (lastLoop == 0L) 0.0 else (now-lastLoop)/1e6)
                    telemetry.update()
                    lastTelemetry = now
                }
                lastLoop = now
            }
        } finally {
            try {
                shooter.power = 0.0
                left.power = 0.0
                right.power = 0.0
                gate.position = ServoPositions.GATE_CLOSED
            } finally {
                hubs.forEachIndexed { i, hub -> hub.bulkCachingMode = oldModes[i] }
            }
        }
    }
}
