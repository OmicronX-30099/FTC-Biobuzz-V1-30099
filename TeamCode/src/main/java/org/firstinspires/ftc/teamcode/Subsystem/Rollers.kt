package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

object Rollers {
    const val INTAKE_POWER = 1.0
    const val SHOOTING_POWER = 0.45
    const val REVERSE_VELOCITY_TPS = -800.0
    var POWER_EPSILON = 0.01
    var rollBool: Boolean = false
    // full Intake includes both rollers & transfer
    var intakePower: Double = 0.0
    var intakeVelocityTPS: Double = 0.0
        private set
    private var velocityControlActive = false
    lateinit var fullIntake1: DcMotorEx
    lateinit var fullIntake2: DcMotorEx
    fun init(hardwareMap: HardwareMap) {

        fullIntake1 = hardwareMap.get(DcMotorEx::class.java, "left_intake")
        fullIntake1.direction = DcMotorSimple.Direction.FORWARD
        fullIntake2 = hardwareMap.get(DcMotorEx::class.java, "right_intake")
        fullIntake2.direction = DcMotorSimple.Direction.REVERSE
        fullIntake1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        fullIntake2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velocityControlActive = false
        intakeVelocityTPS = 0.0
        rollBool = false
        stop()
    }

    fun stop() {
        if (velocityControlActive) {
            setPower(0.0)
            return
        }
        intakePower = 0.0
        fullIntake1.power = 0.0
        fullIntake2.power = 0.0
    }

    /** Shooting takes priority over reverse, then intake; released controls stop both motors. */
    fun update(rightTrigger: Float, leftTrigger: Float, gateOpen: Boolean) {
        when {
            gateOpen -> setPower(SHOOTING_POWER)
            leftTrigger > 0.2f -> setVelocity(REVERSE_VELOCITY_TPS)
            rightTrigger > 0.2f -> setPower(INTAKE_POWER)
            else -> setPower(0.0)
        }
    }

    private fun setVelocity(ticksPerSecond: Double) {
        if (!velocityControlActive) {
            fullIntake1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            fullIntake2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityControlActive = true
            intakeVelocityTPS = 0.0
        }

        intakePower = 0.0
        if (intakeVelocityTPS == ticksPerSecond) return
        fullIntake1.velocity = ticksPerSecond
        fullIntake2.velocity = ticksPerSecond
        intakeVelocityTPS = ticksPerSecond
    }

    // usually gonna be at 1.0 power
    fun changePow(iPow: Double): Command {
        return Command.build()
            .setStart{
                setPower(iPow)
            }
    }

    // Continuous controls can apply power directly without allocating a command each loop.
    @JvmOverloads
    fun setPower(iPow: Double, maxPower: Double = 1.0) {
        val power = iPow.coerceIn(-maxPower, maxPower)
        val wasVelocityControlActive = velocityControlActive
        if (wasVelocityControlActive) {
            fullIntake1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            fullIntake2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            velocityControlActive = false
            intakeVelocityTPS = 0.0
            intakePower = power
            fullIntake1.power = power
            fullIntake2.power = power
            return
        }
        // Enforce a lowered limit even when the change is smaller than POWER_EPSILON.
        interChangePow(power, abs(intakePower) > maxPower)
    }

    private fun interChangePow(iPow: Double, force: Boolean) {
        // Skip insignificant manual changes while always allowing a stop.
        if (iPow == intakePower) return
        if (!force && iPow != 0.0 && abs(intakePower - iPow) <= POWER_EPSILON) return
        intakePower = iPow
        fullIntake1.power = iPow
        fullIntake2.power = iPow
    }
}
