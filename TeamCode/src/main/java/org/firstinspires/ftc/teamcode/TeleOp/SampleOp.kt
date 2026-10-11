package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.SuboSystem.MecanumDrive
import org.firstinspires.ftc.teamcode.SuboSystem.Rollers


@TeleOp(name = "Sample Tele", group = "TeleOp")
public class SampleOp: OpMode() {
    lateinit var driver: MecanumDrive
    lateinit var rollers: Rollers
    override fun init() {
        driver = MecanumDrive()
        rollers = Rollers
    }

    override fun loop() {

        driver.drive( -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        if (gamepad1.right_trigger_pressed) {rollers.run(1.0, 0.25)}
    }

    override fun stop() {
        driver.stop()
    }

}
