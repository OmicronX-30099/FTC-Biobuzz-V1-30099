package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.behaviors.BlockedBehavior
import com.pedropathing.ivy.behaviors.ConflictBehavior
import com.pedropathing.ivy.behaviors.EndCondition
import com.pedropathing.ivy.behaviors.InterruptedBehavior
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController

object Shooter {


    val hood: DcMotor = hardwareMap.get(DcMotor::class.java, "hood")
    val shooter: DcMotor = hardwareMap.get(DcMotor::class.java, "shooter")
    var shooterPow: Double = 0.0;
    var offPow: Double = 0.0
    fun init() {}
    fun changePow(changed: Double): Command {
        return Command.build()
            .setStart{
                shooter.power = changed
            }
    }
    var Shoot: Command = sequential(
        changePow(shooterPow),
        waitMs(250.0),
        Rollers.changePow(0.67, 0.67),
        waitMs(250.0),
        changePow(offPow),
        waitMs(250.0),
        Rollers.changePow(0.0, 0.0)
    )
    fun update() {
        shooterPow = ShooterVelocityController.calculate(1000.0, 1000.0,
            ShooterVelocityController.DEFAULT_KP,
            ShooterVelocityController.DEFAULT_KS,
            ShooterVelocityController.DEFAULT_KV)
        offPow = ShooterVelocityController.calculate(0.0, 1000.0,
            ShooterVelocityController.DEFAULT_KP,
            ShooterVelocityController.DEFAULT_KS,
            ShooterVelocityController.DEFAULT_KV)
    }
}