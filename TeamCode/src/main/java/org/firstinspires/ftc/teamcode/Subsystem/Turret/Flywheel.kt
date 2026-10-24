package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.experimental.physics.PhysicsShot
import org.firstinspires.ftc.teamcode.experimental.physics.PhysicsShooter
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController

@Config
object Flywheel {
    private const val HOOD_COMMAND_EPSILON = 0.001
    private var lastMotorPower = 0.0
    // Reuse one shot object; the LUT lookup performs no per-loop allocations.
    private val physicsShot = PhysicsShot()
    @JvmField @Volatile var AUTO_DISTANCE_SHOT: Boolean = false
    @JvmField @Volatile var TARGET_VELOCITY_TPS: Double = 1460.0

    @JvmField @Volatile var HOOD_POSITION: Double = 0.35
    var hoodPosition: Double = ServoPositions.HOOD_DEFAULT
        private set
    var distanceToGoalInches: Double = Double.NaN
        private set
    var shotStatus: String = "Not started"
        private set
    var rawLutVelocityTPS: Double = Double.NaN
        private set
    var calculatedHoodAngleDegrees: Double = Double.NaN
        private set
    var predictedEntryAngleDegrees: Double = Double.NaN
        private set
    var predictedVertexHeightCm: Double = Double.NaN
        private set
    var lutHeightFeasible: Boolean = false
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
        clearLutDiagnostics()
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
        // Field coordinates are inches; use the 2D horizontal distance, not 3D range.
        distanceToGoalInches = Math.hypot(goalTargetX - pose.x(), goalTargetY - pose.y())

        var requestedHoodPosition = HOOD_POSITION
        val baseVelocityTPS: Double
        if (AUTO_DISTANCE_SHOT) {
            PhysicsShooter.calculateShot(distanceToGoalInches, physicsShot)
            if (!physicsShot.valid || !physicsShot.shooterVelocityTPS.isFinite() ||
                physicsShot.shooterVelocityTPS < 0.0) {
                return stopInvalidShot("Physics LUT invalid/outside 35-75 in - shooter stopped")
            }

            val lutHoodPosition = PhysicsShooter.hoodServoPosition(physicsShot.hoodAngleDegrees)
            if (!lutHoodPosition.isFinite()) {
                return stopInvalidShot("Physics LUT hood angle invalid - shooter stopped")
            }
            // Do not silently clip away a physics-calculated hood setting. The configured
            // servo limits must agree with the LUT's 15-36 degree mechanical mapping.
            val clippedHoodPosition = ServoPositions.clipHoodPosition(lutHoodPosition)
            if (!clippedHoodPosition.isFinite() ||
                kotlin.math.abs(clippedHoodPosition - lutHoodPosition) > 1e-6) {
                return stopInvalidShot("Physics LUT hood outside servo limits - shooter stopped")
            }

            baseVelocityTPS = physicsShot.shooterVelocityTPS
            requestedHoodPosition = lutHoodPosition
            rawLutVelocityTPS = baseVelocityTPS
            calculatedHoodAngleDegrees = physicsShot.hoodAngleDegrees
            predictedEntryAngleDegrees = physicsShot.entryAngleDegrees
            predictedVertexHeightCm = physicsShot.vertexHeightCm
            lutHeightFeasible = physicsShot.heightConstraintSatisfied
            // A valid descending fallback may still fail the desired vertex-height constraint.
            shotStatus = if (physicsShot.feasible) "Physics LUT (provisional): height feasible"
            else "Physics LUT (provisional): height-infeasible fallback"
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

    private fun clearLutDiagnostics() {
        rawLutVelocityTPS = Double.NaN
        calculatedHoodAngleDegrees = Double.NaN
        predictedEntryAngleDegrees = Double.NaN
        predictedVertexHeightCm = Double.NaN
        lutHeightFeasible = false
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
