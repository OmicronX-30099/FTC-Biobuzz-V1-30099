package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.SuboSystem.MecanumDrive


@TeleOp(name = "Sample Tele", group = "TeleOp")
public class SampleOp: OpMode() {
    lateinit var big: MecanumDrive
    override fun init() {
        big = MecanumDrive()
    }

    override fun loop() {
        big.horiLogic(gamepad1.right_trigger)
        TODO("Not yet implemented")
    }

}
