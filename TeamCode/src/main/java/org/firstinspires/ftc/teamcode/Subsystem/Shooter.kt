package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.control.ShooterVelocityController

/**
 * these are some suggestions
 * 1.look at suggestions in the update function
 * 2.for shooting, I dont want it to run a set sequence,
 * make it so that the intake runs at 1.0 as long as trigger is pressed
 * 3.also, changePow in the Shoot command is redundant, power of shooter should
 * be actively adjusted no matter what state the robot is in
 * 4.get turret done quickly
 */
object Shooter {


    val hood: Servo = hardwareMap.get(Servo::class.java, "hood")
    val shooter: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter")
    var shooterPow: Double = 0.0
    var shooterVelocity: Double = 0.0
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
        shooterVelocity = shooter.velocity
        //1000.0 is placeholder, real target will be calculated through a shooting equation
        shooterPow = ShooterVelocityController.calculate(1000.0,
            shooterVelocity,
            ShooterVelocityController.DEFAULT_KP,
            ShooterVelocityController.DEFAULT_KS,
            ShooterVelocityController.DEFAULT_KV)
        //shooter will never be turned off, this just adds extra calculations
        // to each loop, which increases looptime
        offPow = ShooterVelocityController.calculate(0.0,
            shooterVelocity,
            ShooterVelocityController.DEFAULT_KP,
            ShooterVelocityController.DEFAULT_KS,
            ShooterVelocityController.DEFAULT_KV)
    }
}