package org.firstinspires.ftc.teamcode.experimental.physics

/** Driver calibration of y = m*x + b, isolated from the nominal physics model. */
class PhysicsVelocityTrim {
    private var multiplierHundredths = 100
    private var offsetSteps = 0
    private var wasLeft = false
    private var wasRight = false
    private var wasUp = false
    private var wasDown = false

    val multiplier: Double get() = multiplierHundredths / 100.0
    val offsetTPS: Double get() = offsetSteps * 20.0

    fun seedButtons(left: Boolean, right: Boolean, up: Boolean, down: Boolean) {
        wasLeft = left
        wasRight = right
        wasUp = up
        wasDown = down
    }

    fun reset(left: Boolean, right: Boolean, up: Boolean, down: Boolean) {
        multiplierHundredths = 100
        offsetSteps = 0
        seedButtons(left, right, up, down)
    }

    fun update(left: Boolean, right: Boolean, up: Boolean, down: Boolean) {
        val multiplierDelta = (if (right && !wasRight) 1 else 0) -
            (if (left && !wasLeft) 1 else 0)
        val offsetDelta = (if (up && !wasUp) 1 else 0) -
            (if (down && !wasDown) 1 else 0)
        multiplierHundredths = (multiplierHundredths + multiplierDelta).coerceAtLeast(0)
        offsetSteps += offsetDelta
        seedButtons(left, right, up, down)
    }

    fun apply(lutVelocityTPS: Double): Double {
        if (!lutVelocityTPS.isFinite() || lutVelocityTPS < 0) return 0.0
        val adjusted = multiplier * lutVelocityTPS + offsetTPS
        return if (adjusted.isFinite()) adjusted.coerceAtLeast(0.0) else 0.0
    }
}
