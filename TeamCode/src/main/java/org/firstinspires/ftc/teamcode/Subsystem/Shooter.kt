package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Util.ServoPositions

object Shooter {
    enum class Mode { NORMAL, BACKUP }

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
    fun init(hardwareMap: HardwareMap) {
        mode = Mode.NORMAL
        gate = hardwareMap.get(Servo::class.java, "gate")
        gate.direction = Servo.Direction.FORWARD
        Flywheel.init(hardwareMap)
        Turret.init(hardwareMap)
        gate.position = ServoPositions.GATE_CLOSED
        gateOpen = false
        shooterPow = 0.0
        shooterVelocity = 0.0
    }
    fun start() {
        Turret.start()
    }

    fun toggleMode(): Mode {
        mode = if (mode == Mode.NORMAL) Mode.BACKUP else Mode.NORMAL
        return mode
    }

    fun setGate(open: Boolean) {
        if (open == gateOpen) return
        gate.position = if (open) ServoPositions.GATE_OPEN else ServoPositions.GATE_CLOSED
        gateOpen = open
    }

    @JvmOverloads
    fun update(pose: Pose, velocityOffsetTPS: Double = 0.0, aimOffsetDegrees: Double = 0.0) {
        shooterVelocity = shooter.velocity
        if (mode == Mode.BACKUP) {
            shooterPow = Flywheel.updateBackup(pose, shooterVelocity)
            Turret.updateToDegrees(0.0)
        } else {
            shooterPow = Flywheel.update(pose, shooterVelocity, velocityOffsetTPS)
            Turret.update(pose, aimOffsetDegrees)
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
