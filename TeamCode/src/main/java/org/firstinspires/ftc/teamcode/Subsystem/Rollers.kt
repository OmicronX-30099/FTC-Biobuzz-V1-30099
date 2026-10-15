package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Subsystem.Shooter.shooter

object Rollers {
    var rollBool: Boolean = false
    var currRollSpeed: Double = 0.0
    lateinit var intakeRoll: DcMotorEx;
    lateinit var transferRoll: DcMotorEx
    fun init(hardwareMap: HardwareMap) {

        intakeRoll = hardwareMap.get(DcMotorEx::class.java, "intR")
        transferRoll = hardwareMap.get(DcMotorEx::class.java, "trans")
    }
    fun intialize(intakeSpeed: Double, reset: Boolean = false) {
        if (reset) {
            currRollSpeed = 0.0
            rollBool = false
            return
        }
        currRollSpeed = intakeSpeed

    }
    fun changePow(iPow: Double, tPow: Double): Command {
        return Command.build()
            .setStart{
                intakeRoll.power = iPow
                transferRoll.power = tPow
            }
    }

    val RollIntake: Command = Command.build()
        .setStart{
            intakeRoll.power = 1.0

        }
        .setDone{
            rollBool
        }
}