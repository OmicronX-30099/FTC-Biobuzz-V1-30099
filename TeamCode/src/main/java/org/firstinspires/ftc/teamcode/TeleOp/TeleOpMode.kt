package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsystem.MecanumDrive
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.Subsystem.Shooter

@TeleOp(name = "TeleOpBlue", group = "Blue")
class TeleOpMode: OpMode() {

    override fun init() {

    }
    override fun loop() {
        MecanumDrive.drive( -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        if (gamepad1.right_trigger > 0.0 && gamepad1.left_trigger < 0) {Rollers.changePow(1.0).execute()}
        if (gamepad1.left_trigger > 0.0 && gamepad1.right_trigger < 0) {Rollers.changePow(-1.0).execute()}
        if (gamepad1.right_trigger < 0.0 && gamepad1.left_trigger < 0 ) {Rollers.changePow(0.0).execute()}
        if (gamepad1.right_bumper) {if (Shooter.busyGate) return
            Shooter.OpenGate.execute()}
        //if (gamepad1.right_bumper) {
        //    Shooter.Shoot().execute()
        //}
    }
}