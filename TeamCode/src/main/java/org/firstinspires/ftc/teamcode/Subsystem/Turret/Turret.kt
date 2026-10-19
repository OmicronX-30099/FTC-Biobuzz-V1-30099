package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
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
    lateinit var turret: DcMotorEx
        private set
    private var lastTargetTicks = 0
    private var running = false

    fun init(hardwareMap: HardwareMap) {
        turret = hardwareMap.get(DcMotorEx::class.java, "turret")
        turret.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        turret.targetPosition = 0
        turret.mode = DcMotor.RunMode.RUN_TO_POSITION
        turret.power = 0.0
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        lastTargetTicks = 0
        running = false
    }

    fun update(pose: Pose) {
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

        // Re-sending an unchanged target still sends a command to the hub.
        if (targetTicks != lastTargetTicks) {
            turret.targetPosition = targetTicks
            lastTargetTicks = targetTicks
        }
        if (!running) {
            turret.power = 0.4
            running = true
        }

    }

    fun stop() {
        turret.power = 0.0
        running = false
    }
}
