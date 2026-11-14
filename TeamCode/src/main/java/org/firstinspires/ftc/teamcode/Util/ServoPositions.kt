package org.firstinspires.ftc.teamcode.Util

/** Measured logical servo positions, recorded from the servo tester. */
object ServoPositions {
    const val HOOD_DEFAULT = 0.35
    const val HOOD_MIN = 0.16 // 36-degree hood endpoint
    const val HOOD_MAX = 0.485 // 15-degree hood endpoint

    @JvmStatic
    fun clipHoodPosition(position: Double): Double =
        if (position.isFinite()) position.coerceIn(HOOD_MIN, HOOD_MAX) else HOOD_DEFAULT

    const val GATE_CLOSED = 0.87
    const val GATE_OPEN = 0.75
    const val LEFT_FLOWER_DOWN = 0.517
    const val RIGHT_FLOWER_DOWN = 0.50
    const val LEFT_FLOWER_UP = 0.83
    const val RIGHT_FLOWER_UP = 0.82
}
