package org.firstinspires.ftc.teamcode.SuboSystem

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

object Rollers {
    private val intakeMot: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake")
    private val transferMot: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "trans")
    var intakePower: Double = 0.0
    var transferPower: Double = 0.0
    fun init(hardwareMap: HardwareMap) {

    }
    fun intialize() {intakePower = 0.0
    transferPower = 0.0}
    fun run(iPow: Double, tPow: Double) {
        intakePower = iPow
        transferPower = tPow
        intakeMot.power = intakePower
        transferMot.power = transferPower
    }
}