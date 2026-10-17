package org.firstinspires.ftc.teamcode.Subsystem.Turret

import android.transition.TransitionManager.go
import com.pedropathing.config.Configuration
import com.pedropathing.revhub.localizers.PinpointConfig
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret.goal
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.BlueAlliance
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.RedAlliance
import org.firstinspires.ftc.teamcode.pedro.TeleConstants
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin


// turret motor
object Turret {
    var goal: kotlin.Int = 1
    var targetCordX = 0.0
    var targetCordY = 0.0
    // Converting Degrees to Relative Encoder Ticks
    val turret = hardwareMap.get(DcMotorEx::class.java, "turrSer")
    val follower = TeleConstants.follower
    fun intialize() {
        turret.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turret.targetPosition = 0
        turret.mode = DcMotor.RunMode.RUN_TO_POSITION
        turret.power = 0.0
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun update() {
        follower.update()

        val pose = follower.pose()

        val dx = CurrAlliance.CurrGoal.x() - pose.x()
        val dy = CurrAlliance.CurrGoal.y() - pose.y()

        val targetHeading = atan2(dx, dy)

        val turretAngle = atan2(
            sin(targetHeading - pose.heading()),
            cos(targetHeading - pose.heading())
        )

        val limitedAngle = turretAngle.coerceIn(
            Math.toRadians(-90.0),
            Math.toRadians(90.0)
        )

        // Convert turret angle to motor encoder ticks.
        val targetTicks = (
                Math.toDegrees(limitedAngle) / 360.0 * (384.5*4.5)
                ).roundToInt()

        turret.targetPosition = targetTicks
        turret.power = 0.4

    }
}