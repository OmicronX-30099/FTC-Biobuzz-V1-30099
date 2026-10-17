package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.pedro.TeleConstants
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

object Flywheel {
    var goalTargetX: Double = 0.0
    var goalTargetY: Double = 0.0
    const val DEFAULT_KP: Double = 0.003
    const val DEFAULT_KS: Double = 0.04
    const val DEFAULT_KV: Double = 0.00038
    var flyWheelVelo: Double = 0.0
    var prevWheelVelo: Double = 0.0
    val shooter: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter")
    val hood: Servo = hardwareMap.get(Servo::class.java, "hood")

    val follower = TeleConstants.follower
    fun calculate(
        targetVelocity: Double, currentVelocity: Double,
        kP: Double, kS: Double, kV: Double
    ): Double {
        if (!java.lang.Double.isFinite(targetVelocity) || !java.lang.Double.isFinite(currentVelocity) || !java.lang.Double.isFinite(
                kP
            ) || !java.lang.Double.isFinite(kS) || !java.lang.Double.isFinite(kV) || abs(
                targetVelocity
            ) < 1.0
        ) {
            return 0.0
        }

        val error = targetVelocity - currentVelocity
        val feedforward = kV * targetVelocity + kS * sign(targetVelocity)
        val power = feedforward + kP * error
        return if (java.lang.Double.isFinite(power)) max(-1.0, min(1.0, power)) else 0.0
    }
    fun update() {
        follower.update()
        val pose = follower.pose()
        val d = sqrt(
            (pose.x() - CurrAlliance.CurrGoal.x()).pow(2.0) +
                    (pose.y() - CurrAlliance.CurrGoal.y()).pow(2.0)
        )
        flyWheelVelo = ((0.01632 * d * d) + (3.49146 * d) + 985.48178)
        calculate(flyWheelVelo, prevWheelVelo, DEFAULT_KP, DEFAULT_KS, DEFAULT_KV)
        prevWheelVelo = flyWheelVelo
        shooter.power = flyWheelVelo

    }
}