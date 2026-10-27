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
    enum class Mode { NORMAL, BACKUP }

    var sotmEnabled = true
        private set

    var mode = Mode.NORMAL
        private set

    lateinit var gate: Servo
        private set
    var gateOpen: Boolean = false
        private set


    val hood: Servo get() = Flywheel.hood
    val shooter: DcMotorEx get() = Flywheel.shooter
    var shooterPow: Double = 0.0
    var shooterVelocity: Double = 0.0
    var offPow: Double = 0.0
    var shotFlightTimeSeconds: Double = Double.NaN
        private set
    var predictedShotPose: Pose? = null
        private set
    fun init(hardwareMap: HardwareMap) {
        mode = Mode.NORMAL
        sotmEnabled = true
        gate = hardwareMap.get(Servo::class.java, "gate")
        gate.direction = Servo.Direction.FORWARD
        Flywheel.init(hardwareMap)
        Turret.init(hardwareMap)
        gate.position = ServoPositions.GATE_CLOSED
        gateOpen = false
        shooterPow = 0.0
        shooterVelocity = 0.0
        shotFlightTimeSeconds = Double.NaN
        predictedShotPose = null
    }
    fun start() {
        Turret.start()
    }

    fun toggleMode(): Mode {
        mode = if (mode == Mode.NORMAL) Mode.BACKUP else Mode.NORMAL
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

    /** Predicts the robot pose at impact, then uses that pose for turret, distance, TPS and hood. */
    fun updateWithMotion(
        pose: Pose,
        fieldVelocityXInchesPerSecond: Double,
        fieldVelocityYInchesPerSecond: Double,
        angularVelocityRadiansPerSecond: Double,
        velocityOffsetTPS: Double = 0.0,
        aimOffsetDegrees: Double = 0.0
    ) {
        shooterVelocity = shooter.velocity
        val goal = CurrAlliance.CurrGoal
        val currentDistance = Math.hypot(goal.x() - pose.x(), goal.y() - pose.y())
        shotFlightTimeSeconds = FlightTimeInterpolator.atDistance(currentDistance)
        val aimingPose = if (sotmEnabled) {
            ShootingPosePrediction.predict(pose,
                fieldVelocityXInchesPerSecond, fieldVelocityYInchesPerSecond,
                angularVelocityRadiansPerSecond, shotFlightTimeSeconds)
        } else pose
        predictedShotPose = aimingPose
        if (mode == Mode.BACKUP) {
            shooterPow = Flywheel.updateBackup(aimingPose, shooterVelocity, velocityOffsetTPS)
            Turret.updateToDegrees(0.0)
        } else {
            shooterPow = Flywheel.update(aimingPose, shooterVelocity, velocityOffsetTPS)
            Turret.update(aimingPose, aimOffsetDegrees)
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
