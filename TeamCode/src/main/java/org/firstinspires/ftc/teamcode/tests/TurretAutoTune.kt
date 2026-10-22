package org.firstinspires.ftc.teamcode.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ReadWriteFile
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Subsystem.Breakbeams
import org.firstinspires.ftc.teamcode.Subsystem.Flowers
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.Subsystem.Shooter
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Subsystem.Turret.TurretConfig
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.control.TurretAiming
import org.firstinspires.ftc.teamcode.control.TurretAutoTuner
import org.firstinspires.ftc.teamcode.control.TurretPositionController
import org.firstinspires.ftc.teamcode.pedro.Constants

/** Standalone experiment: never writes TurretConfig or calls the competition OpMode. */
@TeleOp(name = "Turret Auto Tuner", group = "Tests")
class TurretAutoTune : OpMode() {
    companion object {
        @JvmField var SWEEP_AMPLITUDE_DEG = 15.0
        @JvmField var STEP_SECONDS = 1.0
        @JvmField var SETTLE_TOLERANCE_DEG = 0.8
        @JvmField var MAX_LOOP_GAP_MS = 150.0
    }

    private lateinit var follower: Follower
    private lateinit var hubs: List<LynxModule>
    private lateinit var originalCacheModes: List<LynxModule.BulkCachingMode>
    private var battery: VoltageSensor? = null
    private var voltage = 12.0
    private var lastBatteryNanos = 0L
    private var lastTelemetryNanos = 0L
    private var previousLoopNanos = 0L
    private var loopMs = 0.0
    private var controlMs = 0.0
    private var tuner: TurretAutoTuner? = null
    private var originalGains: TurretAutoTuner.Gains? = null
    private var settingsSnapshot = emptyList<Double>()
    private var hardwareSnapshot = emptyList<Any>()
    private var oldGoal: Pose? = null
    private var oldAlliance: Alliance? = null
    private var oldUseGoal1 = true
    private var oldGoalWasOverride = false
    private var preview = "OFF"
    private var status = "A: tune; B: stop turret. X: preview winner; Y: preview original"
    private var savedReport = false
    private var aWasPressed = false
    private var xWasPressed = false
    private var yWasPressed = false
    private var circleWasPressed = false
    private var target = 0.0
    private var power = 0.0

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 100
        // Deliberately reproduce TeleOp's workload here; competition code stays untouched.
        follower = Constants.create(hardwareMap)
        resetPose()
        oldGoal = CurrAlliance.CurrGoal
        oldAlliance = CurrAlliance.cur
        oldUseGoal1 = CurrAlliance.useGoal1
        oldGoalWasOverride = oldGoal !== if (oldUseGoal1) CurrAlliance.cur.GoalPose1 else CurrAlliance.cur.GoalPose2
        CurrAlliance.Reset()
        Shooter.init(hardwareMap)
        Rollers.init(hardwareMap)
        Flowers.init(hardwareMap)
        Breakbeams.init(hardwareMap)
        battery = hardwareMap.voltageSensor.firstOrNull()
        hardwareSnapshot = hardwareSettings()
        hubs = hardwareMap.getAll(LynxModule::class.java)
        originalCacheModes = hubs.map { it.bulkCachingMode }
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
        init_loop()
    }

    override fun init_loop() {
        hubs.forEach { it.clearBulkCache() }
        Encoders.seedAtCurrentPosition()
        telemetry.addLine("Standalone tuner; shared gains are never changed.")
        telemetry.addLine("Uses TurretConfig calibration. Check initial angle before START.")
        telemetry.addLine("START runs TeleOp drive/flywheel workload; A begins turret sweeps.")
        telemetry.addData("Estimated turret angle (deg)", Encoders.trueDegrees)
        telemetry.addData("Flywheel target (ticks/s)", Flywheel.TARGET_VELOCITY_TPS)
        telemetry.update()
    }

    override fun start() {
        hubs.forEach { it.clearBulkCache() }
        Shooter.start()
        aWasPressed = gamepad1.a
        xWasPressed = gamepad1.x
        yWasPressed = gamepad1.y
        circleWasPressed = gamepad1.circle
        lastTelemetryNanos = System.nanoTime() - 100_000_000L
    }

    override fun loop() {
        val nowNanos = System.nanoTime()
        val now = nowNanos / 1e9
        if (previousLoopNanos != 0L) loopMs = (nowNanos - previousLoopNanos) / 1e6
        previousLoopNanos = nowNanos
        hubs.forEach { it.clearBulkCache() }
        if (Breakbeams.update(nowNanos)) {
            gamepad1.rumble(200)
            gamepad2.rumble(200)
        }
        if (gamepad1.circle && !circleWasPressed) resetPose()
        circleWasPressed = gamepad1.circle
        follower.manual(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble())
        follower.update()
        val shooting = gamepad1.left_bumper
        Shooter.setGate(shooting)
        Rollers.update(gamepad1.right_trigger, gamepad1.left_trigger, Shooter.gateOpen)
        Flowers.updateToggle(gamepad1.right_bumper)
        Flywheel.update(follower.pose())
        Encoders.update()
        if (nowNanos - lastBatteryNanos >= 250_000_000L) {
            voltage = battery?.voltage ?: 12.0
            lastBatteryNanos = nowNanos
        }

        var beganSweep = false
        if (gamepad1.b) {
            tuner?.cancel()
            preview = "OFF"
            status = "Turret stopped; A: restart"
        } else if (gamepad1.a && !aWasPressed && tuner?.isRunning != true) {
            beginSweep(now)
            beganSweep = true
        } else if (tuner?.isRunning != true) {
            if (gamepad1.x && !xWasPressed && tuner?.state == TurretAutoTuner.State.COMPLETE) preview = "WINNER"
            if (gamepad1.y && !yWasPressed && runComplete()) preview = "ORIGINAL"
        }
        aWasPressed = gamepad1.a
        xWasPressed = gamepad1.x
        yWasPressed = gamepad1.y

        val run = tuner
        if (run != null && (run.isRunning || preview != "OFF") &&
            (settingsSnapshot != controllerSettings() || hardwareSnapshot != hardwareSettings())) {
            run.fail("Calibration/controller/tuning settings changed; stop and INIT again")
            preview = "OFF"
        }
        if (run?.isRunning == true && !beganSweep) run.update(now, Encoders.trueDegrees, Encoders.velocityDegreesPerSecond)
        power = 0.0
        if (run?.isRunning == true) {
            target = run.target
            command(run.commandGains)
        } else if (preview != "OFF" && run?.state == TurretAutoTuner.State.COMPLETE) {
            val pose = follower.pose()
            val goal = CurrAlliance.CurrGoal
            target = TurretAiming.targetDegrees(pose.x(), pose.y(), pose.heading(), goal.x(), goal.y(),
                Encoders.trueDegrees, TurretConfig.MIN_ANGLE_DEG, TurretConfig.MAX_ANGLE_DEG,
                TurretConfig.CLOCKWISE_POSITIVE)
            command(if (preview == "WINNER") run.best else originalGains!!)
        }
        Turret.turret.power = power
        controlMs = (System.nanoTime() - nowNanos) / 1e6

        // File I/O happens only after output has been stopped at the end of a run.
        if (run?.state == TurretAutoTuner.State.COMPLETE && !savedReport) {
            savedReport = true
            try {
                ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("turret-auto-tune.txt"), report(run))
                status = "Result saved to FIRST/settings/turret-auto-tune.txt; copy gains to source to keep them"
            } catch (exception: RuntimeException) {
                status = "Could not save report: ${exception.message}; record gains from telemetry"
            }
        }
        if (nowNanos - lastTelemetryNanos >= 100_000_000L) {
            showTelemetry()
            lastTelemetryNanos = nowNanos
        }
    }

    private fun beginSweep(now: Double) {
        preview = "OFF"
        savedReport = false
        try {
            require(hardwareSnapshot == hardwareSettings()) { "Calibration changed; stop and INIT again" }
            require(TurretConfig.MAX_POWER.isFinite() && TurretConfig.MAX_POWER > 0 && TurretConfig.MAX_POWER <= 1)
            require(TurretConfig.FRICTION_RAMP_DEG.isFinite() && TurretConfig.FRICTION_RAMP_DEG > 0)
            require(TurretConfig.DEADBAND_DEG.isFinite() && TurretConfig.DEADBAND_DEG >= 0)
            require(SETTLE_TOLERANCE_DEG >= TurretConfig.DEADBAND_DEG)
            originalGains = TurretAutoTuner.Gains(TurretConfig.kP, TurretConfig.kD, TurretConfig.kS)
            settingsSnapshot = controllerSettings()
            tuner = TurretAutoTuner(originalGains, TurretConfig.MIN_ANGLE_DEG, TurretConfig.MAX_ANGLE_DEG,
                SWEEP_AMPLITUDE_DEG, STEP_SECONDS, SETTLE_TOLERANCE_DEG, MAX_LOOP_GAP_MS / 1000)
            tuner!!.start(now, Encoders.trueDegrees)
            status = "Sweeps use the current angle as center. B stops turret output immediately"
        } catch (exception: IllegalArgumentException) {
            tuner = null
            status = "Cannot start: ${exception.message ?: "check source settings"}"
        }
    }

    private fun runComplete() = tuner?.state == TurretAutoTuner.State.COMPLETE

    private fun command(gains: TurretAutoTuner.Gains) {
        power = TurretPositionController.calculate(target, Encoders.trueDegrees, Encoders.velocityDegreesPerSecond,
            gains.p, gains.d, gains.s, TurretConfig.FRICTION_RAMP_DEG, TurretConfig.DEADBAND_DEG,
            TurretConfig.MAX_POWER, voltage)
        if ((Encoders.trueDegrees <= TurretConfig.MIN_ANGLE_DEG && power < 0) ||
            (Encoders.trueDegrees >= TurretConfig.MAX_ANGLE_DEG && power > 0)) power = 0.0
    }

    private fun controllerSettings() = listOf(TurretConfig.kP, TurretConfig.kD, TurretConfig.kS,
        TurretConfig.MAX_POWER, TurretConfig.FRICTION_RAMP_DEG, TurretConfig.DEADBAND_DEG,
        TurretConfig.MIN_ANGLE_DEG, TurretConfig.MAX_ANGLE_DEG, SWEEP_AMPLITUDE_DEG,
        STEP_SECONDS, SETTLE_TOLERANCE_DEG, MAX_LOOP_GAP_MS)

    private fun hardwareSettings(): List<Any> = listOf(TurretConfig.MOTOR_NAME, TurretConfig.ABSOLUTE_ENCODER_NAME,
        TurretConfig.RELATIVE_ENCODER_NAME, TurretConfig.MOTOR_REVERSED, TurretConfig.RELATIVE_ENCODER_REVERSED,
        TurretConfig.ABSOLUTE_ENCODER_REVERSED, TurretConfig.TICKS_PER_MOTOR_REV,
        TurretConfig.MOTOR_REVS_PER_TURRET_REV, TurretConfig.ABSOLUTE_FULL_SCALE_VOLTS,
        TurretConfig.ABSOLUTE_ZERO_DEG, TurretConfig.INITIAL_TURRET_ESTIMATE_DEG, TurretConfig.CLOCKWISE_POSITIVE)

    private fun showTelemetry() {
        val run = tuner
        telemetry.addLine("A: auto tune; B: stop turret; X: winner aiming; Y: original aiming")
        telemetry.addLine(status)
        telemetry.addData("Tuner / preview", "${run?.state ?: "IDLE"} / $preview")
        if (run != null) {
            telemetry.addLine(run.message)
            telemetry.addData("Trial", if (run.isValidating) "Validation" else "${run.trial} / ${TurretAutoTuner.SEARCH_TRIALS}")
            telemetry.addData("Candidate P / D / S", "%.6f / %.6f / %.6f", run.commandGains.p, run.commandGains.d, run.commandGains.s)
            telemetry.addData("Best P / D / S", "%.6f / %.6f / %.6f", run.best.p, run.best.d, run.best.s)
            telemetry.addData("Score original / best / repeat (lower is better)", "%.3f / %.3f / %.3f",
                run.baselineScore, run.bestScore, run.validationScore)
            telemetry.addData("Best mean error / overshoot (deg)", "%.2f / %.2f", run.bestMeanError, run.bestOvershoot)
            telemetry.addData("Tuning loop avg / max (ms)", "%.2f / %.2f", run.averageLoopMs, run.maxLoopMs)
        }
        telemetry.addData("Turret angle / target / error (deg)", "%.2f / %.2f / %.2f", Encoders.trueDegrees, target, target - Encoders.trueDegrees)
        telemetry.addData("Turret velocity (deg/s)", Encoders.velocityDegreesPerSecond)
        telemetry.addData("Turret power / cap", "%.3f / %.3f", power, TurretConfig.MAX_POWER)
        telemetry.addData("Loop / control (ms)", "%.2f / %.2f", loopMs, controlMs)
        telemetry.addData("Battery (V)", voltage)
        telemetry.addData("Flywheel target / measured (ticks/s)", "%.1f / %.1f", Flywheel.flyWheelVelo, Flywheel.shooter.velocity)
        telemetry.addData("Shot distance / hood", "%.2f / %.3f", Flywheel.distanceToGoalInches, Flywheel.hoodPosition)
        telemetry.addData("Shot settings", Flywheel.shotStatus)
        telemetry.addData("Robot x / y / heading (deg)", "%.2f / %.2f / %.1f", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()))
        telemetry.addData("Balls", Breakbeams.ballCount)
        telemetry.update()
    }

    private fun report(run: TurretAutoTuner) = """
        Turret automatic tuning result (not automatically applied)
        kP = ${run.best.p}; kD = ${run.best.d}; kS = ${run.best.s};
        Original P/D/S = ${originalGains!!.p} / ${originalGains!!.d} / ${originalGains!!.s}
        Baseline / best / validation score = ${run.baselineScore} / ${run.bestScore} / ${run.validationScore}
        Best mean error / overshoot (deg) = ${run.bestMeanError} / ${run.bestOvershoot}
        Average / maximum loop (ms) = ${run.averageLoopMs} / ${run.maxLoopMs}
        MAX_POWER = ${TurretConfig.MAX_POWER}; FRICTION_RAMP_DEG = ${TurretConfig.FRICTION_RAMP_DEG}; DEADBAND_DEG = ${TurretConfig.DEADBAND_DEG}
        Sweep amplitude / step seconds = $SWEEP_AMPLITUDE_DEG / $STEP_SECONDS
        Set TurretConfig kP/kD/kS manually after comparing X (winner) and Y (original).
        Record these values in TurretConfig.java to persist across robot app restarts.
    """.trimIndent()

    private fun resetPose() { follower.setPose(Pose(8.2, 7.3, Math.toRadians(90.0))) }

    override fun stop() {
        Shooter.stop()
        follower.manual(0.0, 0.0, 0.0)
        follower.update()
        Rollers.stop()
        oldAlliance?.let {
            CurrAlliance.cur = it
            CurrAlliance.SwitchGoal(if (oldUseGoal1) 1 else 2)
            if (oldGoalWasOverride) oldGoal?.let { goal -> CurrAlliance.CurrGoal = goal }
        }
        if (::hubs.isInitialized) hubs.forEachIndexed { i, hub -> hub.bulkCachingMode = originalCacheModes[i] }
    }
}
