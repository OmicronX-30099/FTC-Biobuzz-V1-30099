package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Flywheel
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret

/**
 * these are some suggestions
 * 1.look at suggestions in the update function
 * 2.for shooting, I don't want it to run a set sequence,
 * make it so that the intake runs at 1.0 as long as trigger is pressed
 * 3.also, changePow in the Shoot command is redundant, power of shooter should
 * be actively adjusted no matter what state the robot is in
 * 4.get turret done quickly
 */
object Shooter {
    val gate: Servo = hardwareMap.get(Servo::class.java, "gate")
    var gateOpen: Boolean = false


    val hood: Servo = hardwareMap.get(Servo::class.java, "hood")
    val shooter: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "shooter")
    var shooterPow: Double = 0.0
    var shooterVelocity: Double = 0.0
    var offPow: Double = 0.0
    var busyGate: Boolean = false;
    fun init() {}
    val OpenGate: Command = sequential(
        Command.build().setStart {
            busyGate = true
            gate.position = 0.5
        },
        waitMs(100.0),
        Command.build().setStart {
            busyGate = false
            gate.position = 0.0
        }
    )
    /*
    fun changePow(changed: Double): Command {
        if (busyShooting) return Command.build()
        return Command.build()
            .setStart{
                shooter.power = changed
            }
    }

     */

    fun update() {
        shooterVelocity = shooter.velocity
        //1000.0 is placeholder, real target will be calculated through a shooting equation
        Flywheel.update()
        Turret.update()
        //shooter will never be turned off, this just adds extra calculations
        // to each loop, which increases looptime
    }
}