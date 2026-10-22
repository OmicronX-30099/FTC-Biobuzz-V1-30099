package org.firstinspires.ftc.teamcode.experimental.physics

/** Manual calibration settings, independent of the physics LUT and Dashboard. */
class CalibrationSettings {
    var targetTPS = 1480.0
        private set
    var hoodPosition = 0.30
        private set
    private var wasLeft = false
    private var wasRight = false
    private var wasUp = false
    private var wasDown = false

    fun seedButtons(left: Boolean, right: Boolean, up: Boolean, down: Boolean) {
        wasLeft = left
        wasRight = right
        wasUp = up
        wasDown = down
    }

    fun reset(left: Boolean, right: Boolean, up: Boolean, down: Boolean) {
        targetTPS = 1480.0
        hoodPosition = 0.30
        seedButtons(left, right, up, down)
    }

    fun update(left: Boolean, right: Boolean, up: Boolean, down: Boolean) {
        val hoodDelta = (if (right && !wasRight) 1 else 0) - (if (left && !wasLeft) 1 else 0)
        val speedDelta = (if (up && !wasUp) 1 else 0) - (if (down && !wasDown) 1 else 0)
        hoodPosition = (hoodPosition + hoodDelta*0.005).coerceIn(
            PhysicsShooter.hoodServoPosition(36.0), PhysicsShooter.hoodServoPosition(15.0))
        targetTPS = (targetTPS + speedDelta*20.0).coerceAtLeast(0.0)
        seedButtons(left, right, up, down)
    }
}
