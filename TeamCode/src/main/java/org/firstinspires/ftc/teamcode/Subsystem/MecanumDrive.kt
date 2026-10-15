package org.firstinspires.ftc.teamcode.Subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.max

object MecanumDrive {
    private lateinit var flDrive: DcMotor
    private lateinit var frDrive: DcMotor
    private lateinit var blDrive: DcMotor

    private lateinit var brDrive: DcMotor

    private var flPower = 0.0;
    private var blPower = 0.0;
    private var frPower = 0.0;
    private var brPower = 0.0;

    fun init(hardwareMap: HardwareMap) {
        flDrive = hardwareMap.get(DcMotor::class.java, "flDrive")
        frDrive = hardwareMap.get(DcMotor::class.java, "frDrive")
        blDrive = hardwareMap.get(DcMotor::class.java, "blDrive")
        brDrive = hardwareMap.get(DcMotor::class.java, "brDrive")

        frDrive.direction = DcMotorSimple.Direction.REVERSE
        brDrive.direction = DcMotorSimple.Direction.REVERSE

    }
    fun drive(forward: Double, strafe: Double, rotate: Double) {
        // forward =
        var strafe = strafe
        strafe *= 1.1


        // Keep all powers within [-1, 1] while preserving their ratios
        val denominator = max(
            abs(forward) + abs(strafe) + abs(rotate), 1.0
        )

        flPower = (forward + strafe + rotate) / denominator
        blPower = (forward - strafe + rotate) / denominator
        frPower = (forward - strafe - rotate) / denominator
        brPower = (forward + strafe - rotate) / denominator

        flDrive.power = flPower
        frDrive.power = frPower
        blDrive.power = blPower
        brDrive.power = brPower
    }
    fun stop() {
        drive(0.0, 0.0, 0.0)
    }

    fun setBrake(brake: Boolean) {
        val behavior = if (brake)
            ZeroPowerBehavior.BRAKE
        else
            ZeroPowerBehavior.FLOAT
        frDrive.setZeroPowerBehavior(behavior)
        blDrive.setZeroPowerBehavior(behavior)
        frDrive.setZeroPowerBehavior(behavior)
        brDrive.setZeroPowerBehavior(behavior)
    }


}
