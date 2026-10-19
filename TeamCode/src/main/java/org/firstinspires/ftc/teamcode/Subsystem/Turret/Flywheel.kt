package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController
import kotlin.math.sqrt

object Flywheel {
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
        hood.position = ServoPositions.HOOD_DEFAULT
        shooter.power = 0.0
        shooter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        flyWheelVelo = 0.0
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

    fun update(pose: Pose, currentVelocity: Double): Double {
        val goal = CurrAlliance.CurrGoal
        val dx = pose.x() - goal.x()
        val dy = pose.y() - goal.y()
        val d = sqrt(dx * dx + dy * dy)
        flyWheelVelo = 1500.0//((0.01632 * d * d) + (3.49146 * d) + 985.48178)
        val power = calculate(flyWheelVelo, currentVelocity, DEFAULT_KP, DEFAULT_KS, DEFAULT_KV)
        shooter.power = power
        return power
    }

    fun stop() {
        shooter.power = 0.0
        flyWheelVelo = 0.0
    }
}
