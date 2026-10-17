package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

// Subsystem for both absolute & Relative encoders for the turret
// 72 rotations of the relative encoders is equal to 16 rotations of the turret
// Absolute encoders goes from 0 which is 0 degrees to 3.3 Volts, which is 360 degrees
// 384.5 ticks per rotation for relative encoder
object Encoders {
    // "absEn" refers to absolute encoders
    // "relEn" refers to relative encoders
    lateinit var absEncoder: AnalogInput
    lateinit var relEncoder: DcMotor
    var trueDegrees: Double = 0.0
    var rot: Int = 0

    fun init(hardwareMap: HardwareMap) {
        absEncoder = hardwareMap.get(AnalogInput::class.java, "absEn")
        relEncoder = hardwareMap.get(DcMotor::class.java, "frDrive")

    }

    fun update() {
        rot = (relEncoder.currentPosition/(384.5*6)).toInt()
        trueDegrees = rot*360 + (absEncoder.voltage * (360/3.3))
    }
    fun EncodeToDegree(EncodeTick: Double): Double {
        return 360/1730.45 * EncodeTick
    }


}