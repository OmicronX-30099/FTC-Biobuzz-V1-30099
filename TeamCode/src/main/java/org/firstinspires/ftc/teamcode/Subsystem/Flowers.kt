package org.firstinspires.ftc.teamcode.Subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Util.ServoPositions

object Flowers {
    private lateinit var left: Servo
    private var bumperWasPressed = false

    var isDown = false
        private set

    fun init(hardwareMap: HardwareMap) {
        left = hardwareMap.get(Servo::class.java, "left_flower")
        left.direction = Servo.Direction.FORWARD
        isDown = false
        bumperWasPressed = false
        applyPositions()
    }

    fun updateToggle(bumperPressed: Boolean) {
        if (bumperPressed && !bumperWasPressed) {
            isDown = !isDown
            applyPositions()
        }
        bumperWasPressed = bumperPressed
    }

    private fun applyPositions() {
        left.position = if (isDown) ServoPositions.LEFT_FLOWER_DOWN else ServoPositions.LEFT_FLOWER_UP
    }
}
