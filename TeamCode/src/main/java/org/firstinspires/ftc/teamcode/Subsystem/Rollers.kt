package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
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
    var POWER_EPSILON = 0.001
    var rollBool: Boolean = false
    var busyShooting: Boolean = false
    // full Intake includes both rollers & transfer
    var intakePower: Double = 0.0
    lateinit var fullIntake1: DcMotorEx
    lateinit var fullIntake2: DcMotorEx
    fun init(hardwareMap: HardwareMap) {

        fullIntake1 = hardwareMap.get(DcMotorEx::class.java, "int1")
        fullIntake1.direction = DcMotorSimple.Direction.REVERSE
        fullIntake2 = hardwareMap.get(DcMotorEx::class.java, "int2")
    }
    fun intialize(intakeSpeed: Double, reset: Boolean = false) {
        if (reset) {
            intakePower = 0.0
            rollBool = false
            return
        }
        intakePower = intakeSpeed
    }
    // usually gonna be at 1.0 power
    fun changePow(iPow: Double): Command {
        if (busyShooting) return Command.build()
        return Command.build()
            .setStart{
                interChangePow(iPow)
            }
    }
    private fun interChangePow(iPow: Double) {
        // internal change power
        if (abs(intakePower - iPow) <= POWER_EPSILON) return
        intakePower = iPow
        fullIntake1.power = iPow
        fullIntake2.power = iPow
    }
}