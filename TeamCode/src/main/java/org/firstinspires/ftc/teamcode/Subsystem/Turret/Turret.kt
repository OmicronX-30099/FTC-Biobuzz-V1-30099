package org.firstinspires.ftc.teamcode.Subsystem.Turret

import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.control.TurretAiming
import org.firstinspires.ftc.teamcode.control.TurretPositionController

object Turret {
    lateinit var turret: DcMotorEx
        private set
    private var battery: VoltageSensor? = null
    private var active = false
    private var batteryVoltage = 12.0
    private var lastBatteryReadNanos = 0L
    var targetDegrees = 0.0
        private set
    var motorPower = 0.0
        private set

    fun init(hardwareMap: HardwareMap) {
        active = false
        turret = hardwareMap.get(DcMotorEx::class.java, TurretConfig.MOTOR_NAME)
        turret.power = 0.0
        turret.direction = if (TurretConfig.MOTOR_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
        turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        // frDrive supplies the turret encoder signal; do not reset or command it here.
        Encoders.init(hardwareMap)
        battery = hardwareMap.voltageSensor.firstOrNull()
        batteryVoltage = 12.0
        lastBatteryReadNanos = 0L
        targetDegrees = Encoders.trueDegrees
        motorPower = 0.0
    }

    fun start() {
        // Re-seed after INIT in case the turret was moved before START.
        Encoders.seedAtCurrentPosition()
        targetDegrees = Encoders.trueDegrees
        active = true
    }

    fun update(pose: Pose) {
        if (!active) return
        Encoders.update()
        val now = System.nanoTime()
        if (now - lastBatteryReadNanos >= 250_000_000L) {
            val measured = battery?.voltage ?: 12.0
            if (measured.isFinite() && measured > 0.0) batteryVoltage = measured
            lastBatteryReadNanos = now
        }
        val current = Encoders.trueDegrees
        val goal = CurrAlliance.CurrGoal
        val minAngle = TurretConfig.MIN_ANGLE_DEG
        val maxAngle = TurretConfig.MAX_ANGLE_DEG
        targetDegrees = TurretAiming.targetDegrees(
            pose.x(), pose.y(), pose.heading(), goal.x(), goal.y(), current, minAngle, maxAngle,
            TurretConfig.CLOCKWISE_POSITIVE
        )
        var power = TurretPositionController.calculate(
            targetDegrees, current, Encoders.velocityDegreesPerSecond,
            TurretConfig.kP, TurretConfig.kD, TurretConfig.kS,
            TurretConfig.FRICTION_RAMP_DEG, TurretConfig.DEADBAND_DEG,
            TurretConfig.MAX_POWER, batteryVoltage
        )
        if ((current <= minAngle && power < 0) || (current >= maxAngle && power > 0)) power = 0.0
        // Avoid repeated SDK commands at saturation/deadband without dropping small corrections.
        if (power != motorPower) turret.power = power
        motorPower = power
    }

    fun stop() {
        active = false
        motorPower = 0.0
        if (::turret.isInitialized) turret.power = 0.0
    }
}
