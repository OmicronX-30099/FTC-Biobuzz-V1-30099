package org.firstinspires.ftc.teamcode.SuboSystem

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

object ShooterIntiation {
    public var flywb: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "flywb")
    private var flywVelo = 0.0
    private var flywVeloOff = 0.0
    private var flywVelcurr = 0.0

    fun init (hardwareMap: HardwareMap) {

    }
    fun CalcVelocity(d: Double) {flywVelo = d*10 + 1000}
    fun setOff(d: Double) {flywVeloOff = d}

}