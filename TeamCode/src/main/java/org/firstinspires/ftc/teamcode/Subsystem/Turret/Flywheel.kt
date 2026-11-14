package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.control.DistanceShotInterpolator
import org.firstinspires.ftc.teamcode.control.DistanceShotModel
import org.firstinspires.ftc.teamcode.control.FlywheelVelocityKalman
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController
import kotlin.math.exp

@Config
object Flywheel {
    private const val HOOD_COMMAND_EPSILON = 0.001
    private const val REFERENCE_VOLTAGE = 12.0
    private const val BATTERY_SAMPLE_INTERVAL_NS = 100_000_000L
    private const val BATTERY_FILTER_TIME_CONSTANT_SECONDS = 3.0
    const val BACKUP_TARGET_VELOCITY_TPS = 1600.0
    const val BACKUP_HOOD_POSITION = 0.27
    private var lastMotorPower = 0.0
    private var batterySensor: VoltageSensor? = null
    private var lastBatterySampleNanos = 0L
    private var previousVoltage = Double.NaN
    private var olderVoltage = Double.NaN
    var rawBatteryVoltage = Double.NaN
        private set
    var filteredBatteryVoltage = Double.NaN
        private set
    private val distanceShot = DistanceShotInterpolator.Shot()
    private val velocityFilter = FlywheelVelocityKalman()
    @JvmField @Volatile var VOLTAGE_COMPENSATION_ENABLED: Boolean = false
    @JvmField @Volatile var AUTO_DISTANCE_SHOT: Boolean = true
    @JvmField @Volatile var TARGET_VELOCITY_TPS: Double = 1600.0
    @JvmField @Volatile var KALMAN_FILTER_ENABLED: Boolean = false
    @JvmField @Volatile var KALMAN_PROCESS_NOISE_TPS: Double = 20.0
    @JvmField @Volatile var KALMAN_MEASUREMENT_NOISE_TPS: Double = 30.0

    @JvmField @Volatile var HOOD_POSITION: Double = ServoPositions.HOOD_DEFAULT
    var hoodPosition: Double = ServoPositions.HOOD_DEFAULT
        private set
    var distanceToGoalInches: Double = Double.NaN
        private set
    var shotStatus: String = "Not started"
        private set
    var calibratedVelocityTPS: Double = Double.NaN
        private set
    var shotClamped: Boolean = false
        private set

    var goalTargetX: Double = 0.0
    var goalTargetY: Double = 0.0
    const val DEFAULT_KP: Double = 0.003
    const val DEFAULT_KS: Double = 0.06
    const val DEFAULT_KV: Double = 0.000357
    // Live Dashboard gains shared by manual, distance, backup, and auto shots.
    @JvmField @Volatile var KP: Double = DEFAULT_KP
    @JvmField @Volatile var KS: Double = DEFAULT_KS
    @JvmField @Volatile var KV: Double = DEFAULT_KV
    var flyWheelVelo: Double = 0.0
    lateinit var shooter: DcMotorEx
        private set
    lateinit var hood: Servo
        private set

    fun init(hardwareMap: HardwareMap) {
        shooter = hardwareMap.get(DcMotorEx::class.java, "shooter")
        hood = hardwareMap.get(Servo::class.java, "hood")
        hood.direction = Servo.Direction.FORWARD
        hoodPosition = ServoPositions.clipHoodPosition(HOOD_POSITION)
        hood.position = hoodPosition
        shooter.power = 0.0
        lastMotorPower = 0.0
        shooter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        shooter.direction = DcMotorSimple.Direction.REVERSE
        batterySensor = hardwareMap.voltageSensor.firstOrNull()
        previousVoltage = Double.NaN
        olderVoltage = Double.NaN
        filteredBatteryVoltage = Double.NaN
        updateBatteryVoltage(sampleImmediately = true)
        resetVelocityFilter(shooter.velocity)
        flyWheelVelo = 0.0
        distanceToGoalInches = Double.NaN
        shotStatus = "Not started"
        clearLutDiagnostics()
    }

    fun calculate(
        targetVelocity: Double, currentVelocity: Double,
        kP: Double, kS: Double, kV: Double
    ): Double {
        return ShooterVelocityController.calculate(targetVelocity, currentVelocity, kP, kS, kV)
    }
    fun update(pose: Pose) {
        update(pose, filterVelocity(shooter.velocity))
    }

    /** Use raw encoder feedback unless Kalman filtering is explicitly enabled. */
    fun filterVelocity(measuredVelocityTPS: Double): Double {
        if (!KALMAN_FILTER_ENABLED) {
            resetVelocityFilter(measuredVelocityTPS)
            return measuredVelocityTPS
        }
        return velocityFilter.update(measuredVelocityTPS,
            KALMAN_PROCESS_NOISE_TPS, KALMAN_MEASUREMENT_NOISE_TPS)
    }

    /** Seed the estimate at INIT so startup does not ramp up from an assumed zero. */
    fun resetVelocityFilter(initialVelocityTPS: Double) {
        val measurementNoise = if (KALMAN_MEASUREMENT_NOISE_TPS.isFinite() &&
            KALMAN_MEASUREMENT_NOISE_TPS > 0.0) KALMAN_MEASUREMENT_NOISE_TPS else 30.0
        velocityFilter.reset(initialVelocityTPS, measurementNoise * measurementNoise)
    }

    @JvmOverloads
    fun update(pose: Pose, currentVelocity: Double, velocityOffsetTPS: Double = 0.0): Double {
        updateDistanceToGoal(pose)

        var requestedHoodPosition = HOOD_POSITION
        val baseVelocityTPS: Double
        if (AUTO_DISTANCE_SHOT) {
            if (!DistanceShotModel.sampleInto(distanceToGoalInches,
                    ShotCalibration.DISTANCE_INCHES,
                    ShotCalibration.HOOD_POSITIONS, distanceShot)) {
                return stopInvalidShot("Distance calibration invalid - shooter stopped")
            }
            baseVelocityTPS = distanceShot.velocityTPS
            requestedHoodPosition = distanceShot.hoodPosition
            calibratedVelocityTPS = baseVelocityTPS
            shotClamped = distanceShot.clamped
            shotStatus = if (distanceShot.clamped) "Polynomial velocity; hood endpoint held"
            else "Polynomial velocity; interpolated hood"
        } else {
            clearLutDiagnostics()
            baseVelocityTPS = TARGET_VELOCITY_TPS
            shotStatus = "Manual"
        }

        if (!baseVelocityTPS.isFinite() || baseVelocityTPS < 0.0 ||
            !velocityOffsetTPS.isFinite()) {
            return stopInvalidShot("Invalid shooter TPS/trim - shooter stopped")
        }
        // Use the requested velocity directly so a fixed target remains exactly 1460 TPS.
        flyWheelVelo = (baseVelocityTPS + velocityOffsetTPS).coerceAtLeast(0.0)
        if (!flyWheelVelo.isFinite()) {
            return stopInvalidShot("Shooter TPS overflow - shooter stopped")
        }

        setHoodPosition(requestedHoodPosition)
        return updateMotorPower(currentVelocity)
    }

    fun updateBackup(pose: Pose, currentVelocity: Double, velocityOffsetTPS: Double = 0.0): Double {
        updateDistanceToGoal(pose)
        clearLutDiagnostics()
        if (!velocityOffsetTPS.isFinite()) {
            return stopInvalidShot("Invalid shooter trim - shooter stopped")
        }
        flyWheelVelo = (BACKUP_TARGET_VELOCITY_TPS + velocityOffsetTPS).coerceAtLeast(0.0)
        shotStatus = "Backup: ${flyWheelVelo.toInt()} TPS / hood $BACKUP_HOOD_POSITION"
        setHoodPosition(BACKUP_HOOD_POSITION)
        return updateMotorPower(currentVelocity)
    }
    fun updateAuto(pose: Pose, currentVelocity: Double, NewVelocity: Double, NewHood: Double): Double {
        updateDistanceToGoal(pose)
        clearLutDiagnostics()
        if (!NewVelocity.isFinite() || !NewHood.isFinite()) {
            return stopInvalidShot("Invalid shooter trim - shooter stopped")
        }
        flyWheelVelo = (NewVelocity).coerceAtLeast(0.0)
        setHoodPosition(NewHood)
        shotStatus = "Auto: ${flyWheelVelo.toInt()} TPS / hood $NewHood"
        return updateMotorPower(currentVelocity)
    }

    private fun updateMotorPower(currentVelocity: Double): Double {
        updateBatteryVoltage()
        val voltageScale = if (VOLTAGE_COMPENSATION_ENABLED && filteredBatteryVoltage.isFinite()) {
            REFERENCE_VOLTAGE / filteredBatteryVoltage
        } else 1.0
        val power = ShooterVelocityController.calculate(flyWheelVelo, currentVelocity,
            KP, KS, KV, voltageScale)
        applyMotorPower(power)
        return power
    }

    private fun updateBatteryVoltage(sampleImmediately: Boolean = false) {
        val now = System.nanoTime()
        val elapsedNanos = now - lastBatterySampleNanos
        if (!sampleImmediately && elapsedNanos < BATTERY_SAMPLE_INTERVAL_NS) return
        lastBatterySampleNanos = now

        val sample = batterySensor?.voltage ?: Double.NaN
        rawBatteryVoltage = sample
        // Discard missing readings and glitches outside the usable range of a 12 V pack.
        if (sample !in 8.0..16.0) return
        if (!olderVoltage.isFinite()) {
            // Collect three valid samples before using the battery reading for control.
            olderVoltage = previousVoltage
            previousVoltage = sample
            return
        }

        // The median drops isolated spikes. The slow average smooths sustained changes.
        val median = maxOf(minOf(previousVoltage, olderVoltage),
            minOf(maxOf(previousVoltage, olderVoltage), sample))
        olderVoltage = previousVoltage
        previousVoltage = sample
        if (!filteredBatteryVoltage.isFinite()) {
            filteredBatteryVoltage = median
            return
        }
        // Cap elapsed time so resuming after a pause cannot cause a large correction.
        val elapsedSeconds = elapsedNanos.coerceAtMost(2 * BATTERY_SAMPLE_INTERVAL_NS) / 1e9
        val alpha = 1.0 - exp(-elapsedSeconds / BATTERY_FILTER_TIME_CONSTANT_SECONDS)
        filteredBatteryVoltage += alpha * (median - filteredBatteryVoltage)
    }

    private fun updateDistanceToGoal(pose: Pose) {
        val goal = CurrAlliance.CurrGoal
        goalTargetX = goal.x()
        goalTargetY = goal.y()
        // Field coordinates are inches; use the 2D horizontal distance, not 3D range.
        distanceToGoalInches = Math.hypot(goalTargetX - pose.x(), goalTargetY - pose.y())
    }

    private fun setHoodPosition(requestedPosition: Double) {
        if (!requestedPosition.isFinite()) return
        val position = ServoPositions.clipHoodPosition(requestedPosition)
        if (kotlin.math.abs(position - hoodPosition) >= HOOD_COMMAND_EPSILON) {
            hood.position = position
            hoodPosition = position
        }
    }

    private fun clearLutDiagnostics() {
        calibratedVelocityTPS = Double.NaN
        shotClamped = false
    }

    private fun stopInvalidShot(reason: String): Double {
        flyWheelVelo = 0.0
        clearLutDiagnostics()
        shotStatus = reason
        // Keep the last valid hood position, but stop the flywheel on invalid data.
        applyMotorPower(0.0)
        return 0.0
    }

    private fun applyMotorPower(power: Double) {
        // Keep every feedback correction; only skip exactly identical commands.
        if (power != lastMotorPower) {
            shooter.power = power
            lastMotorPower = power
        }
    }

    fun stop() {
        shooter.power = 0.0
        lastMotorPower = 0.0
        flyWheelVelo = 0.0
    }
}
