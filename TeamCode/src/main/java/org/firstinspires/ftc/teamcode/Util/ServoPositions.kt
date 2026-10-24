package org.firstinspires.ftc.teamcode.Util

/** Measured logical servo positions, recorded from the servo tester. */
object ServoPositions {
    const val HOOD_15_DEGREES = 0.485
    const val HOOD_DEFAULT = 0.35
    const val HOOD_MAX = 0.16 //36degrees i think

    @JvmStatic
    fun clipHoodPosition(position: Double): Double =
        if (position.isFinite()) position.coerceIn(HOOD_MAX, HOOD_15_DEGREES) else HOOD_DEFAULT

    const val GATE_CLOSED = 0.87
    const val GATE_OPEN = 0.75
    const val LEFT_FLOWER_DOWN = 0.51
    const val RIGHT_FLOWER_DOWN = 0.5
    const val LEFT_FLOWER_UP = 0.83
    const val RIGHT_FLOWER_UP = 0.82
}
