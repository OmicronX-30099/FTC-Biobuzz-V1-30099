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
    var relativeDegreesAtCalibration = Double.NaN
        private set
    var absoluteDegreesAtCalibration = Double.NaN
        private set
    var calibrationErrorDegrees = Double.NaN
        private set
    var largestCalibrationErrorDegrees = 0.0
        private set
    var calibrationAccepted = false
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
        relativeDegreesAtCalibration = Double.NaN
        absoluteDegreesAtCalibration = Double.NaN
        calibrationErrorDegrees = Double.NaN
        largestCalibrationErrorDegrees = 0.0
        calibrationAccepted = false
        seedAtCurrentPosition()
    }

    fun seedAtCurrentPosition() {
        readAbsolute()
        val ticks = relEncoder.currentPosition
        tracker.seed(absoluteMotorDegrees, ticks, initialEstimate)
        trueDegrees = tracker.positionDegrees(ticks)
        velocityDegreesPerSecond = 0.0
    }

    @JvmOverloads
    fun update(recalibrateFromAbsolute: Boolean = false) {
        val ticks = relEncoder.currentPosition
        val now = System.nanoTime()
        if (now - lastAbsoluteReadNanos >= 100_000_000L) {
            readAbsolute()
            if (recalibrateFromAbsolute) {
                relativeDegreesAtCalibration = tracker.positionDegrees(ticks)
                absoluteDegreesAtCalibration = tracker.estimateDegrees(absoluteMotorDegrees,
                    relativeDegreesAtCalibration)
                calibrationErrorDegrees = absoluteDegreesAtCalibration - relativeDegreesAtCalibration
                if (calibrationErrorDegrees.isFinite() &&
                    kotlin.math.abs(calibrationErrorDegrees) > kotlin.math.abs(largestCalibrationErrorDegrees)) {
                    largestCalibrationErrorDegrees = calibrationErrorDegrees
                }
                calibrationAccepted = tracker.correctFromAbsolute(absoluteMotorDegrees, ticks,
                    TurretConfig.BACKUP_MAX_CORRECTION_DEG)
            }
        }
        trueDegrees = tracker.positionDegrees(ticks)
        velocityDegreesPerSecond = tracker.velocityDegreesPerSecond(relEncoder.velocity)
    }

    private fun readAbsolute() {
        absoluteVoltage = absEncoder.voltage
        absoluteMotorDegrees = absoluteVoltage / fullScaleVolts * 360.0
        lastAbsoluteReadNanos = System.nanoTime()
    }
}
