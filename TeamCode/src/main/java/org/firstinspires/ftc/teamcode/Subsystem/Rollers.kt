package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.IntakeVelocityController
import kotlin.math.abs

object Rollers {
    @JvmField @Volatile var GATE_OPEN_TPS = 1100.0
    var POWER_EPSILON = 0.01
    var rollBool: Boolean = false
    // full Intake includes both rollers & transfer
    var intakePower: Double = 0.0
    var targetVelocityTPS: Double = 0.0
        private set
    var measuredVelocityTPS: Double = 0.0
        private set
    lateinit var fullIntake1: DcMotorEx
    lateinit var fullIntake2: DcMotorEx
    fun init(hardwareMap: HardwareMap) {

        fullIntake1 = hardwareMap.get(DcMotorEx::class.java, "left_intake")
        fullIntake1.direction = DcMotorSimple.Direction.FORWARD
        fullIntake2 = hardwareMap.get(DcMotorEx::class.java, "right_intake")
        fullIntake2.direction = DcMotorSimple.Direction.REVERSE
        fullIntake1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        fullIntake2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rollBool = false
        measuredVelocityTPS = 0.0
        stop()
    }

    fun stop() {
        targetVelocityTPS = 0.0
        intakePower = 0.0
        fullIntake1.power = 0.0
        fullIntake2.power = 0.0
    }

    /** Left encoder supplies feedback; both intake motors receive the same power. */
    fun update(rightTrigger: Float, leftTrigger: Float, gateOpen: Boolean) {
        targetVelocityTPS = when {
            gateOpen -> GATE_OPEN_TPS
            leftTrigger > 0.2f -> -1200.0
            rightTrigger > 0.2f -> 2800.0
            else -> 0.0
        }
        measuredVelocityTPS = fullIntake1.velocity
        val power = IntakeVelocityController.calculate(
            targetVelocityTPS, measuredVelocityTPS,
            IntakeVelocityController.DEFAULT_KP, IntakeVelocityController.DEFAULT_KS,
            IntakeVelocityController.DEFAULT_KV
        )
        // Preserve small feedback corrections instead of applying the manual power epsilon.
        interChangePow(power, true)
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
        targetVelocityTPS = 0.0
        val power = iPow.coerceIn(-maxPower, maxPower)
        // Enforce a lowered limit even when the change is smaller than POWER_EPSILON.
        interChangePow(power, abs(intakePower) > maxPower)
    }

    private fun interChangePow(iPow: Double, force: Boolean) {
        // Manual changes use the epsilon; velocity feedback corrections bypass it.
        if (iPow == intakePower) return
        if (!force && iPow != 0.0 && abs(intakePower - iPow) <= POWER_EPSILON) return
        intakePower = iPow
        fullIntake1.power = iPow
        fullIntake2.power = iPow
    }
}
