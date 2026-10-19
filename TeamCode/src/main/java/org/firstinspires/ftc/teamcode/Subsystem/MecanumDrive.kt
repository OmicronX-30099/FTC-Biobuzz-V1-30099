package org.firstinspires.ftc.teamcode.Subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.max

object MecanumDrive {
    private lateinit var flDrive: DcMotorEx
    private lateinit var frDrive: DcMotorEx
    private lateinit var blDrive: DcMotorEx

    private lateinit var brDrive: DcMotorEx

    private var flPower = 0.0;
    private var blPower = 0.0;
    private var frPower = 0.0;
    private var brPower = 0.0;

    fun init(hardwareMap: HardwareMap) {
        flDrive = hardwareMap.get(DcMotorEx::class.java, "flDrive")
        frDrive = hardwareMap.get(DcMotorEx::class.java, "frDrive")
        blDrive = hardwareMap.get(DcMotorEx::class.java, "blDrive")
        brDrive = hardwareMap.get(DcMotorEx::class.java, "brDrive")

        flDrive.direction = DcMotorSimple.Direction.REVERSE
        blDrive.direction = DcMotorSimple.Direction.REVERSE
        frDrive.direction = DcMotorSimple.Direction.FORWARD
        brDrive.direction = DcMotorSimple.Direction.FORWARD

        for (motor in arrayOf(flDrive, frDrive, blDrive, brDrive)) {
            motor.power = 0.0
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        flPower = 0.0
        frPower = 0.0
        blPower = 0.0
        brPower = 0.0
    }
    fun drive(forward: Double, strafe: Double, rotate: Double) {
        // forward =
        var strafe = strafe
        strafe *= 1.1


        // Keep all powers within [-1, 1] while preserving their ratios
        val denominator = max(
            abs(forward) + abs(strafe) + abs(rotate), 1.0
        )

        val nextFlPower = (forward + strafe + rotate) / denominator
        val nextBlPower = (forward - strafe + rotate) / denominator
        val nextFrPower = (forward - strafe - rotate) / denominator
        val nextBrPower = (forward + strafe - rotate) / denominator

        // Exact comparisons retain full joystick resolution while skipping unchanged SDK calls.
        if (nextFlPower != flPower) {
            flDrive.power = nextFlPower
            flPower = nextFlPower
        }
        if (nextFrPower != frPower) {
            frDrive.power = nextFrPower
            frPower = nextFrPower
        }
        if (nextBlPower != blPower) {
            blDrive.power = nextBlPower
            blPower = nextBlPower
        }
        if (nextBrPower != brPower) {
            brDrive.power = nextBrPower
            brPower = nextBrPower
        }
    }
    fun stop() {
        flDrive.power = 0.0
        frDrive.power = 0.0
        blDrive.power = 0.0
        brDrive.power = 0.0
        flPower = 0.0
        frPower = 0.0
        blPower = 0.0
        brPower = 0.0
    }

    fun setBrake(brake: Boolean) {
        val behavior = if (brake)
            ZeroPowerBehavior.BRAKE
        else
            ZeroPowerBehavior.FLOAT
        frDrive.zeroPowerBehavior = behavior
        blDrive.zeroPowerBehavior = behavior
        flDrive.zeroPowerBehavior = behavior
        brDrive.zeroPowerBehavior = behavior
    }


}
