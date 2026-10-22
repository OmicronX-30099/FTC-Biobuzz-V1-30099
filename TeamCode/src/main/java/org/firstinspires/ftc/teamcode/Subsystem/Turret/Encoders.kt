package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.control.TurretEncoderTracker

object Encoders {
    lateinit var absEncoder: AnalogInput
        private set
    lateinit var relEncoder: DcMotorEx
        private set
    private lateinit var tracker: TurretEncoderTracker
    private var fullScaleVolts = 3.3
    private var initialEstimate = 0.0
    private var lastAbsoluteReadNanos = 0L

    var trueDegrees = 0.0
        private set
    var velocityDegreesPerSecond = 0.0
        private set
    var absoluteVoltage = 0.0
        private set
    var absoluteMotorDegrees = 0.0
        private set

    fun init(hardwareMap: HardwareMap) {
        absEncoder = hardwareMap.get(AnalogInput::class.java, TurretConfig.ABSOLUTE_ENCODER_NAME)
        relEncoder = hardwareMap.get(DcMotorEx::class.java, TurretConfig.RELATIVE_ENCODER_NAME)
        fullScaleVolts = TurretConfig.ABSOLUTE_FULL_SCALE_VOLTS
        require(fullScaleVolts.isFinite() && fullScaleVolts > 0)
        initialEstimate = TurretConfig.INITIAL_TURRET_ESTIMATE_DEG
        require(initialEstimate.isFinite())
        tracker = TurretEncoderTracker(
            TurretConfig.TICKS_PER_MOTOR_REV, TurretConfig.MOTOR_REVS_PER_TURRET_REV,
            TurretConfig.ABSOLUTE_ZERO_DEG, TurretConfig.ABSOLUTE_ENCODER_REVERSED,
            TurretConfig.RELATIVE_ENCODER_REVERSED
        )
        seedAtCurrentPosition()
    }

    fun seedAtCurrentPosition() {
        readAbsolute()
        val ticks = relEncoder.currentPosition
        tracker.seed(absoluteMotorDegrees, ticks, initialEstimate)
        trueDegrees = tracker.positionDegrees(ticks)
        velocityDegreesPerSecond = 0.0
    }

    fun update() {
        // Absolute angle seeds startup; relative feedback remains fresh every loop.
        val now = System.nanoTime()
        if (now - lastAbsoluteReadNanos >= 100_000_000L) readAbsolute()
        trueDegrees = tracker.positionDegrees(relEncoder.currentPosition)
        velocityDegreesPerSecond = tracker.velocityDegreesPerSecond(relEncoder.velocity)
    }

    private fun readAbsolute() {
        absoluteVoltage = absEncoder.voltage
        absoluteMotorDegrees = absoluteVoltage / fullScaleVolts * 360.0
        lastAbsoluteReadNanos = System.nanoTime()
    }
}
