package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.control.DistanceShotInterpolator
import org.firstinspires.ftc.teamcode.control.FlywheelVelocityKalman
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController

@Config
object Flywheel {
    private const val HOOD_COMMAND_EPSILON = 0.001
    const val BACKUP_TARGET_VELOCITY_TPS = 1480.0
    const val BACKUP_HOOD_POSITION = 0.25
    private var lastMotorPower = 0.0
    private val distanceShot = DistanceShotInterpolator.Shot()
    private val velocityFilter = FlywheelVelocityKalman()
    @JvmField @Volatile var AUTO_DISTANCE_SHOT: Boolean = true
    @JvmField @Volatile var TARGET_VELOCITY_TPS: Double = 1460.0
    @JvmField @Volatile var KALMAN_PROCESS_NOISE_TPS: Double = 20.0
    @JvmField @Volatile var KALMAN_MEASUREMENT_NOISE_TPS: Double = 30.0

    @JvmField @Volatile var HOOD_POSITION: Double = 0.35
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
    const val DEFAULT_KS: Double = 0.04
    const val DEFAULT_KV: Double = 0.00038
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

    /** Kalman-filter a raw encoder velocity sample before feeding velocity control. */
    fun filterVelocity(measuredVelocityTPS: Double): Double {
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
            if (!DistanceShotInterpolator.sampleInto(distanceToGoalInches,
                    ShotCalibration.DISTANCE_INCHES, ShotCalibration.VELOCITY_TPS,
                    ShotCalibration.HOOD_POSITIONS, distanceShot)) {
                return stopInvalidShot("Distance calibration invalid - shooter stopped")
            }
            baseVelocityTPS = distanceShot.velocityTPS
            requestedHoodPosition = distanceShot.hoodPosition
            calibratedVelocityTPS = baseVelocityTPS
            shotClamped = distanceShot.clamped
            shotStatus = if (distanceShot.clamped) "Distance calibration: endpoint held"
            else "Distance calibration: interpolated"
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
        val power = calculate(flyWheelVelo, currentVelocity, DEFAULT_KP, DEFAULT_KS, DEFAULT_KV)
        applyMotorPower(power)
        return power
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
        val power = calculate(flyWheelVelo, currentVelocity, DEFAULT_KP, DEFAULT_KS, DEFAULT_KV)
        applyMotorPower(power)
        return power
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
