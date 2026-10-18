package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

// Subsystem for both absolute & Relative encoders for the turret
// 72 rotations of the relative encoders is equal to 16 rotations of the turret
// Absolute encoders goes from 0 which is 0 degrees to 3.3 Volts, which is 360 degrees
//Relative encoder is inbuilt to the motor, and is belted to the turret, so direction of motion isn't reversed
//Absolute encoder is geared 1:1 to the motor, so direction of motion is reversed
// 384.5 ticks per rotation for relative encoder
object Encoders {
    lateinit var absEncoder: AnalogInput //absolute encoder
    lateinit var relEncoder: DcMotorEx //relative encoder

    private const val PPR = 384.5 //ticks per revolution of motor
    private const val GEAR_RATIO = 4.5 //turret to motor gear ratio

    // Calibrate this value for your actual encoder installation.
    // It is the absolute encoder angle when the turret is at 0°.
    private const val ABS_ZERO_DEGREES = 0.0

    private var initialTicks = 0
    private var initialDegrees = 0.0

    var trueDegrees = 0.0 //true angle of turret
        private set

    fun init(hardwareMap: HardwareMap) {
        absEncoder = hardwareMap.get(
            AnalogInput::class.java, "absEn"
        )
        relEncoder = hardwareMap.get(
            DcMotorEx::class.java, "frDrive"
        )

        initialTicks = relEncoder.currentPosition

        val absDegrees = absEncoder.voltage / 3.3 * 360.0 //absolute encoder data in degrees

        // Absolute encoder rotates opposite to turret motion.
        // Normalize to [-180°, 180°).
        val difference = (
                (ABS_ZERO_DEGREES - absDegrees + 540.0) % 360.0
                ) - 180.0

        // Convert motor-side angle to turret angle.
        initialDegrees = difference / GEAR_RATIO

        trueDegrees = initialDegrees //this is the initial angle of turret
    }

    fun update() {
        //difference in relative encoder reading from start
        val deltaTicks = relEncoder.currentPosition - initialTicks

        //updated turret angle
        trueDegrees = initialDegrees +
                deltaTicks * 360.0 / (PPR * GEAR_RATIO)
    }
}