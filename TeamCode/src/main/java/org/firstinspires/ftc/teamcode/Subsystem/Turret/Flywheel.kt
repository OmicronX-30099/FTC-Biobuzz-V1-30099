package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.control.DistanceShotInterpolator
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController

object Flywheel {
    private const val HOOD_COMMAND_EPSILON = 0.001
    private var lastMotorPower = 0.0
    private val shot = DistanceShotInterpolator.Shot()
    @JvmField @Volatile var AUTO_DISTANCE_SHOT: Boolean = true
    @JvmField
    @Volatile
    var TARGET_VELOCITY_TPS: Double = 1500.0

    @JvmField @Volatile var HOOD_POSITION: Double = ServoPositions.HOOD_DEFAULT
    var hoodPosition: Double = ServoPositions.HOOD_DEFAULT
        private set
    var distanceToGoalInches: Double = Double.NaN
        private set
    var shotStatus: String = "Not started"
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
        flyWheelVelo = 0.0
        distanceToGoalInches = Double.NaN
        shotStatus = "Not started"
    }

    fun calculate(
        targetVelocity: Double, currentVelocity: Double,
        kP: Double, kS: Double, kV: Double
    ): Double {
        return ShooterVelocityController.calculate(targetVelocity, currentVelocity, kP, kS, kV)
    }
    fun update(pose: Pose) {
        update(pose, shooter.velocity)
    }

    @JvmOverloads
    fun update(pose: Pose, currentVelocity: Double, velocityOffsetTPS: Double = 0.0): Double {
        val goal = CurrAlliance.CurrGoal
        goalTargetX = goal.x()
        goalTargetY = goal.y()
        distanceToGoalInches = Math.hypot(goalTargetX - pose.x(), goalTargetY - pose.y())
        var requestedHoodPosition = HOOD_POSITION
        if (AUTO_DISTANCE_SHOT) {
            if (!DistanceShotInterpolator.sampleInto(distanceToGoalInches,
                ShotCalibration.DISTANCE_INCHES, ShotCalibration.VELOCITY_TPS,
                ShotCalibration.HOOD_POSITIONS, shot)) {
                // Preserve the last valid hood command and stop the flywheel on invalid data.
                flyWheelVelo = 0.0
                shotStatus = "Invalid pose/calibration - shooter stopped"
                applyMotorPower(0.0)
                return 0.0
            }
            flyWheelVelo = shot.velocityTPS
            requestedHoodPosition = shot.hoodPosition
            shotStatus = if (shot.clamped) "Auto - outside measured range, endpoint held" else "Auto - interpolated"
        } else {
            flyWheelVelo = TARGET_VELOCITY_TPS
            shotStatus = "Manual"
        }
        // Apply trim after selecting the base shot; negative trim cannot reverse the shooter.
        flyWheelVelo = (flyWheelVelo + velocityOffsetTPS).coerceAtLeast(0.0)
        if (requestedHoodPosition.isFinite()) {
            val position = ServoPositions.clipHoodPosition(requestedHoodPosition)
            if (kotlin.math.abs(position - hoodPosition) >= HOOD_COMMAND_EPSILON) {
                hood.position = position
                hoodPosition = position
            }
        }
        val power = calculate(flyWheelVelo, currentVelocity, DEFAULT_KP, DEFAULT_KS, DEFAULT_KV)
        applyMotorPower(power)
        return power
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
