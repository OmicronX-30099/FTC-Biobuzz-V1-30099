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
import org.firstinspires.ftc.teamcode.Subsystem.Shooter.ShootCmd
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController

object Shooter {


    val hood: DcMotor = hardwareMap.get(DcMotor::class.java, "hood")
    val shooter: DcMotor = hardwareMap.get(DcMotor::class.java, "shooter")
    var shooterPow: Double = 0.0;
    var offPow: Double = 0.0
    var busyShooting: Boolean = false;
    fun init() {}
    fun changePow(changed: Double): Command {
        if (busyShooting) return Command.build()
        return Command.build()
            .setStart{
                shooter.power = changed
            }
    }
    fun Shoot(): Command {
        if (busyShooting) return Command.build()
        return ShootCmd
    }
    private var ShootCmd: Command = sequential(
        Command.build() .setStart { busyShooting = true
            Rollers.busyShooting = true},
        changePow(shooterPow),
        waitMs(250.0),
        Rollers.changePow(1.0),
        waitMs(250.0),
        changePow(offPow),
        waitMs(250.0),
        Rollers.changePow(0.0),
        Command.build() .setStart { busyShooting = false
            Rollers.busyShooting = false}
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