package org.firstinspires.ftc.teamcode.Auto

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.pedro.Constants

@Autonomous(name = "Red Auto Test", group = "Red")
class RedAuto16P: OpMode()  {
    override fun init() {
        Scheduler.reset()
        follower.setPose(start)
    }

    override fun start() {
        schedule(fullPathRed)
        super.start()
    }
    override fun loop() {
        follower.update()
        Scheduler.execute()

        telemetry.addData("x", follower.pose().x())
        telemetry.addData("y", follower.pose().y())
        telemetry.addData("heading", follower.pose().heading())

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint())
            telemetry.addData("Path number", follower.pathIndex())
        }

        telemetry.update()
    }
    val follower = Constants.create(hardwareMap)
    private val poseFactory = PoseFactory.degrees()

    private val start = poseFactory.of(56.0, 7.3, 90.0)
    private val path1 = poseFactory.of(12.0, 7.3, 180.0)
    private val point2Start = poseFactory.of(12.0, 7.3, 180.0)
    private val point2 = poseFactory.of(24.0, 11.0, 90.0)
    private val point3 = poseFactory.of(24.0, 80.0, 90.0)
    private val point4 = poseFactory.of(48.0, 115.0, 36.1373)
    private val point4Control1 = poseFactory.of(24.3341, 97.8072, 0.0)
    private val openPollenIntakeStart = poseFactory.of(48.0, 115.0, 35.0)
    private val openPollenIntake = poseFactory.of(48.0, 123.0, 90.0)
    private val point6 = poseFactory.of(48.0, 130.0, 90.0)
    private val point7 = poseFactory.of(29.0, 122.0, 90.0)
    private val point7Control1 = poseFactory.of(30.2702, 124.8823, 0.0)
    private val point8 = poseFactory.of(27.0, 61.5, 90.0)
    private val point9Start = poseFactory.of(27.0, 61.5, 90.0)
    private val point9 = poseFactory.of(27.0, 48.5, 180.0)
    private val openPollenIntake2 = poseFactory.of(13.0572, 48.5, 180.0)
    private val point11Start = poseFactory.of(13.0572, 48.5, 180.0)
    private val point11 = poseFactory.of(27.3655, 57.0258, 90.0)
    private val point12 = poseFactory.of(28.0, 111.0, 90.0)
    private val point13Start = poseFactory.of(28.0, 111.0, 90.0)
    private val point13 = poseFactory.of(43.0, 121.0942, 0.0)
    private val point14 = poseFactory.of(8.9731, 105.4529, 0.0)

    fun path1(): Path = Paths.line(start, path1).constant(path1)

    fun path2(): Path = Paths.line(point2Start, point2).linear(point2Start, point2)

    fun path3(): Path = Paths.line(point2, point3).constant(point3)

    fun path4(): Path = Paths.curve(point3, point4Control1, point4).tangent()

    fun openPollenIntake(): Path = Paths.line(openPollenIntakeStart, openPollenIntake).linear(openPollenIntakeStart, openPollenIntake)

    fun path6(): Path = Paths.line(openPollenIntake, point6).constant(point6)

    fun path7(): Path = Paths.curve(point6, point7Control1, point7).constant(point7)

    fun path8(): Path = Paths.line(point7, point8).constant(point8)

    fun path9(): Path = Paths.line(point9Start, point9).linear(point9Start, point9)

    fun openPollenIntake2(): Path = Paths.line(point9, openPollenIntake2).constant(openPollenIntake2)

    fun path11(): Path = Paths.line(point11Start, point11).linear(point11Start, point11)

    fun path12(): Path = Paths.line(point11, point12).constant(point12)

    fun path13(): Path = Paths.line(point13Start, point13).linear(point13Start, point13)

    fun path14(): Path = Paths.line(point13, point14).constant(point14)
    fun followPath(pedroPath: Path): Command {
        return Command.build()
            .setStart() {
                follower.follow(pedroPath)
            }
    }
    val fullPathRed: Command = sequential(
        follow(follower, path1()),
        follow(follower, path2()),
        follow(follower, path3()),
        follow(follower, path4()),
        follow(follower, openPollenIntake()),
        follow(follower, path6()),
        follow(follower, path7()),
        follow(follower, path8()),
        follow(follower, path9()),
        follow(follower, openPollenIntake2()),
        follow(follower, path11()),
        follow(follower, path12()),
        follow(follower, path13()),
        follow(follower, path14())

    )

}