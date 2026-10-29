package org.firstinspires.ftc.teamcode.Util

import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

object BreakBeams {
    val breakB1: DigitalChannel = hardwareMap.get(DigitalChannel::class.java, "bb1")
    val breakB2: DigitalChannel = hardwareMap.get(DigitalChannel::class.java, "bb2")
    val breakB3: DigitalChannel = hardwareMap.get(DigitalChannel::class.java, "bb3")
    val breakB4: DigitalChannel = hardwareMap.get(DigitalChannel::class.java, "bb4")
    var beginRumble: Boolean = false
    fun intialize() {
        breakB1.mode = DigitalChannel.Mode.INPUT
        breakB2.mode = DigitalChannel.Mode.INPUT
        breakB3.mode = DigitalChannel.Mode.INPUT
        breakB4.mode = DigitalChannel.Mode.INPUT
    }
    fun checkForBlock(): Boolean {
        if (!breakB1.state && !breakB2.state && !breakB3.state && !breakB4.state) {
            return true
        }
        return false
    }
}