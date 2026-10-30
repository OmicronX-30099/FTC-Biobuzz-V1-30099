package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.control.FlightTimeInterpolator
import org.firstinspires.ftc.teamcode.control.ShootingPosePrediction

object Shooter {
    enum class Mode { NORMAL, BACKUP, AUTO}

    var sotmEnabled = true
        private set

    var mode = Mode.NORMAL
        private set
    var forceTurretZero = false

    lateinit var gate: Servo
        private set
    var gateOpen: Boolean = false
        private set


    val hood: Servo get() = Flywheel.hood
    val shooter: DcMotorEx get() = Flywheel.shooter
    var autoFlywheel: Double = 0.0
    var autoHood: Double = 0.0
    var shooterPow: Double = 0.0
    var rawShooterVelocity: Double = 0.0
        private set
    var shooterVelocity: Double = 0.0
        private set
    var offPow: Double = 0.0
    var shotFlightTimeSeconds: Double = Double.NaN
        private set
    var predictedShotPose: Pose? = null
        private set
    fun init(hardwareMap: HardwareMap) {
        mode = Mode.NORMAL
        forceTurretZero = false
        sotmEnabled = true
        gate = hardwareMap.get(Servo::class.java, "gate")
        gate.direction = Servo.Direction.FORWARD
        Flywheel.init(hardwareMap)
        rawShooterVelocity = shooter.velocity
        Flywheel.resetVelocityFilter(rawShooterVelocity)
        shooterVelocity = if (rawShooterVelocity.isFinite()) rawShooterVelocity else 0.0
        Turret.init(hardwareMap)
        gate.position = ServoPositions.GATE_CLOSED
        gateOpen = false
        shooterPow = 0.0
        shotFlightTimeSeconds = Double.NaN
        predictedShotPose = null
    }
    fun start() {
        Turret.start()
    }

    fun toggleMode(newMode: Mode): Mode {
        mode = newMode
        return mode
    }

    fun toggleSotm(): Boolean {
        sotmEnabled = !sotmEnabled
        return sotmEnabled
    }

    fun setGate(open: Boolean) {
        if (open == gateOpen) return
        gate.position = if (open) ServoPositions.GATE_OPEN else ServoPositions.GATE_CLOSED
        gateOpen = open
    }

    @JvmOverloads
    fun update(pose: Pose, velocityOffsetTPS: Double = 0.0, aimOffsetDegrees: Double = 0.0) {
        updateWithMotion(pose, 0.0, 0.0, 0.0, velocityOffsetTPS, aimOffsetDegrees)
    }

    /** Refines time of flight against predicted range, then selects the shot at impact. */
    fun updateWithMotion(
        pose: Pose,
        fieldVelocityXInchesPerSecond: Double,
        fieldVelocityYInchesPerSecond: Double,
        angularVelocityRadiansPerSecond: Double,
        velocityOffsetTPS: Double = 0.0,
        aimOffsetDegrees: Double = 0.0
    ) {
        rawShooterVelocity = shooter.velocity
        shooterVelocity = Flywheel.filterVelocity(rawShooterVelocity)
        val goal = CurrAlliance.CurrGoal
        val currentDistance = Math.hypot(goal.x() - pose.x(), goal.y() - pose.y())
        var flightTime = FlightTimeInterpolator.atDistance(currentDistance)
        if (sotmEnabled && flightTime.isFinite() && flightTime >= 0.0 &&
            fieldVelocityXInchesPerSecond.isFinite() && fieldVelocityYInchesPerSecond.isFinite()) {
            // Fixed-point refinement accounts for the changed range during projectile flight.
            repeat(4) {
                val futureX = ShootingPosePrediction.clampFieldCoordinate(
                    pose.x() + fieldVelocityXInchesPerSecond * flightTime)
                val futureY = ShootingPosePrediction.clampFieldCoordinate(
                    pose.y() + fieldVelocityYInchesPerSecond * flightTime)
                val futureDistance = Math.hypot(goal.x() - futureX, goal.y() - futureY)
                val refinedTime = FlightTimeInterpolator.atDistance(futureDistance)
                if (refinedTime.isFinite() && refinedTime >= 0.0) flightTime = refinedTime
            }
        }
        shotFlightTimeSeconds = flightTime
        val aimingPose = if (sotmEnabled) {
            ShootingPosePrediction.predict(pose,
                fieldVelocityXInchesPerSecond, fieldVelocityYInchesPerSecond,
                angularVelocityRadiansPerSecond, flightTime)
        } else pose
        predictedShotPose = aimingPose
        if (mode == Mode.BACKUP) {
            shooterPow = Flywheel.updateBackup(aimingPose, shooterVelocity, velocityOffsetTPS)
            Turret.updateToDegrees(0.0, recalibrateFromAbsolute = true)
        }
        else if (mode == Mode.AUTO) {
            shooterPow = Flywheel.updateAuto(aimingPose, shooterVelocity, autoFlywheel, autoHood)
            if (forceTurretZero) Turret.updateToDegrees(0.0)
            else Turret.update(aimingPose, aimOffsetDegrees)
        }
        else
        {
            shooterPow = Flywheel.update(aimingPose, shooterVelocity, velocityOffsetTPS)
            if (forceTurretZero) Turret.updateToDegrees(0.0)
            else Turret.update(aimingPose, aimOffsetDegrees)
        }
    }

    fun stop() {
        Flywheel.stop()
        Turret.stop()
        gate.position = ServoPositions.GATE_CLOSED
        gateOpen = false
        shooterPow = 0.0
    }
}
