package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

//435 * 16 teeth / 72 teeth for the turret
// breakbeam, has four sensors that utilize light to check if there is balls or not
// absolute encoder, checks the degrees of the degree or circle angle of the turret, blue line and red line means absolute 0, degree doesn't go beyond 360
// has a 1:1 gear, every rotation of the moter every 72 rotations of the
// relative encoder, can go beyond 360, however can be offset
object Rollers {
    var POWER_EPSILON = 0.01
    var rollBool: Boolean = false
    // full Intake includes both rollers & transfer
    var intakePower: Double = 0.0
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
        stop()
    }

    fun stop() {
        intakePower = 0.0
        fullIntake1.power = 0.0
        fullIntake2.power = 0.0
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
        // Enforce a lowered limit even when the change is smaller than POWER_EPSILON.
        interChangePow(power, abs(intakePower) > maxPower)
    }

    private fun interChangePow(iPow: Double, force: Boolean) {
        // internal change power
        //only changes intake power when power difference is greater than POWER_EPSILON
        if (iPow == intakePower) return
        if (!force && iPow != 0.0 && abs(intakePower - iPow) <= POWER_EPSILON) return
        intakePower = iPow
        fullIntake1.power = iPow
        fullIntake2.power = iPow
    }
}
