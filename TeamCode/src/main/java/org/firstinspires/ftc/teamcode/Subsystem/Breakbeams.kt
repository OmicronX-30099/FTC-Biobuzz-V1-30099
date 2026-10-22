package org.firstinspires.ftc.teamcode.Subsystem

import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.BallOccupancy

object Breakbeams {
    // Configure names before INIT; polarity and debounce can be adjusted live.
    @JvmField var BEAM_1_NAME = "bb1"
    @JvmField var BEAM_2_NAME = "bb2"
    @JvmField var BEAM_3_NAME = "bb3"
    @JvmField var BEAM_4_NAME = "bb4"
    @JvmField var BLOCKED_WHEN_LOW = true
    @JvmField var DEBOUNCE_MS = 40.0
    private lateinit var beams: Array<DigitalChannel>
    private var occupancy = BallOccupancy()
    val ballCount: Int get() = occupancy.count

    fun init(hardwareMap: HardwareMap) {
        beams = arrayOf(BEAM_1_NAME, BEAM_2_NAME, BEAM_3_NAME, BEAM_4_NAME).map {
            hardwareMap.get(DigitalChannel::class.java, it).apply { mode = DigitalChannel.Mode.INPUT }
        }.toTypedArray()
        occupancy = BallOccupancy()
    }

    fun update(nowNanos: Long): Boolean {
        val debounce = if (DEBOUNCE_MS.isFinite()) DEBOUNCE_MS.coerceAtLeast(0.0) else 40.0
        for (i in beams.indices) {
            val state = beams[i].state
            occupancy.sample(i, if (BLOCKED_WHEN_LOW) !state else state,
                nowNanos, (debounce * 1_000_000).toLong())
        }
        return occupancy.finishSample()
    }
}
