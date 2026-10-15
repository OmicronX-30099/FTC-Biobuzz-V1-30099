package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive

class TeleOpMode: OpMode() {

    override fun init() {

    }
    override fun loop() {
        MecanumDrive.drive( -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
    }
}