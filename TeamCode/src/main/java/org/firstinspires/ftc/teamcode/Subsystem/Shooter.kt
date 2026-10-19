package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.pedro.TeleConstants

object Shooter {
    // Temporarily disabled; enable here to restore the turret's full lifecycle.
    private const val TURRET_ENABLED = false

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
        gate = hardwareMap.get(Servo::class.java, "gate")
        gate.direction = Servo.Direction.FORWARD
        Flywheel.init(hardwareMap)
        if (TURRET_ENABLED) Turret.init(hardwareMap)
        gate.position = ServoPositions.GATE_CLOSED
        gateOpen = false
        shooterPow = 0.0
        shooterVelocity = 0.0
    }
    fun setGate(open: Boolean) {
        if (open == gateOpen) return
        gate.position = if (open) ServoPositions.GATE_OPEN else ServoPositions.GATE_CLOSED
        gateOpen = open
    }

    fun update() {
        // Only update localization: follower.update() would also command the drive motors.
        TeleConstants.follower.localizer.update()
        update(TeleConstants.follower.pose())
    }

    fun update(pose: Pose) {
        shooterVelocity = shooter.velocity
        shooterPow = Flywheel.update(pose, shooterVelocity)
        if (TURRET_ENABLED) Turret.update(pose)
    }

    fun stop() {
        Flywheel.stop()
        if (TURRET_ENABLED) Turret.stop()
        gate.position = ServoPositions.GATE_CLOSED
        gateOpen = false
        shooterPow = 0.0
    }
}
