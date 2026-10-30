package org.firstinspires.ftc.teamcode.Auto

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.ivy.pedro.PedroCommands.hold
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.interpolator.Interpolator
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.OpModeStorage
import org.firstinspires.ftc.teamcode.Subsystem.Breakbeams
import org.firstinspires.ftc.teamcode.Subsystem.Flowers
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.Subsystem.Shooter
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders
import org.firstinspires.ftc.teamcode.Subsystem.Turret.ShotCalibration
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.control.DistanceShotInterpolator
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.hypot

@Autonomous(name = "redAutoV2", group = "Red")
class RedAutoV2 : OpMode() {
    private lateinit var follower: Follower
    private lateinit var routine: Command
    private lateinit var hubs: List<LynxModule>
    private lateinit var originalCacheModes: List<LynxModule.BulkCachingMode>
    private var shooterInitialized = false
    private var rollersInitialized = false
    private var shooting = false
    private var intaking = false
    private var mechanismsEnabled = false
    private var shotTargetValid = false
    private var autoStartNanos = 0L
    private var turretParkStarted = false

    override fun init() {
        Scheduler.reset()
        shooting = false
        intaking = false
        mechanismsEnabled = false
        shotTargetValid = false
        autoStartNanos = 0L
        turretParkStarted = false
        follower = Constants.create(hardwareMap)
        follower.setPose(start)
        CurrAlliance.Reset()
        Shooter.init(hardwareMap)
        shooterInitialized = true
        Shooter.toggleMode(Shooter.Mode.AUTO)
        Shooter.autoFlywheel = 0.0
        Shooter.autoHood = ServoPositions.HOOD_DEFAULT
        Rollers.init(hardwareMap)
        rollersInitialized = true
        Flowers.init(hardwareMap)
        Breakbeams.init(hardwareMap)
        hubs = hardwareMap.getAll(LynxModule::class.java)
        originalCacheModes = hubs.map { it.bulkCachingMode }
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        routine = sequential(
            waitMs(2000.0),
            shootFor(1000.0),
            instant { setShotTarget(shootcornernear) },
            setIntaking(true),
            follow(follower, intakecorner()),
            follow(follower, shootcornernear()),
            setIntaking(false),
            waitMs(500.0),
            shootFor(150.0),
            instant { setShotTarget(shootcornerfar) },
            follow(follower, shootcornerfar()),
            waitMs(500.0),
            shootFor(300.0),
            // All remaining shots are also at (34, 130); retain the far-corner target.
            setIntaking(true),
            followWithFlowers(intaketopflower(), down = true),
            hold(follower, intaketopflower),
            waitMs(1000.0),
            setIntaking(false),
            followWithFlowers(shoottopflower(), down = false),
            waitMs(500.0),
            shootFor(1200.0),
            follow(follower, intakesideflowerpart1()),
            setIntaking(true),
            followWithFlowers(intakesideflowerpart2(), down = true),
            hold(follower, intakesideflowerpart2),
            waitMs(1000.0),
            setIntaking(false),
            followWithFlowers(shootsideflowerpart1(), down = false),
            follow(follower, shootsideflowerpart2()),
            waitMs(500.0),
            shootFor(1200.0),
            follow(follower, park()),
            finishAuto()
        )
    }

    override fun start() {
        hubs.forEach { it.clearBulkCache() }
        autoStartNanos = System.nanoTime()
        setShotTarget(start)
        Shooter.start()
        mechanismsEnabled = true
        Shooter.update(follower.pose())
        Scheduler.schedule(routine)
    }

    override fun loop() {
        hubs.forEach { it.clearBulkCache() }
        follower.update()
        Scheduler.execute()
        Breakbeams.update(System.nanoTime())
        val pose = follower.pose()
        CurrAlliance.SelectGoalForY(pose.y())
        if (!turretParkStarted && autoStartNanos != 0L &&
            System.nanoTime() - autoStartNanos >= 27_500_000_000L) {
            Shooter.forceTurretZero = true
            Turret.parkAtZero()
            turretParkStarted = true
        }
        if (mechanismsEnabled) {
            Shooter.update(pose)
            Shooter.setGate(shooting)
            Rollers.update(if (intaking) 1.0f else 0.0f, 0.0f, Shooter.gateOpen)
        } else if (turretParkStarted) {
            Turret.updateToDegrees(0.0)
        }
        telemetry.addData("Position", "%.1f / %.1f", pose.x(), pose.y())
        telemetry.addData("Shooting / intaking", "%s / %s", shooting, intaking)
        telemetry.addData("Flower", if (Flowers.isDown) "Down" else "Up")
        telemetry.addData("Flywheel target / measured TPS", "%.1f / %.1f",
            Shooter.autoFlywheel, Shooter.shooterVelocity)
        telemetry.addData("Turret target / angle / power", "%.1f / %.1f / %.2f",
            Turret.targetDegrees, Encoders.trueDegrees, Turret.motorPower)
        telemetry.addData("Balls", Breakbeams.ballCount)
        telemetry.update()
    }

    override fun stop() {
        if (::follower.isInitialized) OpModeStorage.autonomousEndPose = follower.pose()
        Scheduler.reset()
        shooting = false
        intaking = false
        mechanismsEnabled = false
        try {
            if (shooterInitialized) Shooter.stop()
        } finally {
            try {
                if (rollersInitialized) Rollers.stop()
            } finally {
                if (::hubs.isInitialized && ::originalCacheModes.isInitialized) {
                    hubs.forEachIndexed { i, hub -> hub.bulkCachingMode = originalCacheModes[i] }
                }
            }
        }
    }

    private fun shootFor(durationMs: Double): Command = sequential(
        instant { shooting = shotTargetValid },
        waitMs(durationMs),
        instant { shooting = false }
    )

    private fun followWithFlowers(path: Path, down: Boolean): Command = parallel(
        follow(follower, path),
        sequential(waitMs(300.0), instant { Flowers.setDown(down) })
    )

    private fun setIntaking(active: Boolean): Command = instant { intaking = active }

    private fun setShotTarget(point: Pose) {
        CurrAlliance.SelectGoalForY(point.y())
        val goal = CurrAlliance.CurrGoal
        val shot = DistanceShotInterpolator.sample(
            hypot(goal.x() - point.x(), goal.y() - point.y()),
            ShotCalibration.DISTANCE_INCHES, ShotCalibration.VELOCITY_TPS,
            ShotCalibration.HOOD_POSITIONS
        )
        shotTargetValid = shot != null
        Shooter.autoFlywheel = shot?.velocityTPS ?: 0.0
        Shooter.autoHood = shot?.hoodPosition ?: ServoPositions.HOOD_DEFAULT
    }

    private fun finishAuto(): Command = instant {
        shooting = false
        intaking = false
        Shooter.stop()
        if (turretParkStarted) Turret.parkAtZero()
        Rollers.stop()
        mechanismsEnabled = false
    }

    private val poseFactory = PoseFactory.degrees()
    private val start = poseFactory.of(54.4, 7.3, 180.0)
    private val intakecorner = poseFactory.of(10.3, 7.3, 180.0)
    private val shootcornernear = poseFactory.of(34.0, 7.3, 180.0)
    private val shootcornerfar = poseFactory.of(34.0, 130.0, 90.0)
    private val shootcornerfarSegment1Heading = poseFactory.of(34.0, 130.0, 180.0)
    private val shootcornerfarSegment2Start = poseFactory.of(34.0, 130.0, 180.0)
    private val shootcornerfarSegment2End = poseFactory.of(34.0, 130.0, 90.0)
    private val shootcornerfarSegment3Heading = poseFactory.of(34.0, 130.0, 90.0)
    private val intaketopflower = poseFactory.of(48.3, 126.8, 90.0)
    private val intaketopflowerControl1 = poseFactory.of(34.0, 100.0, 0.0)
    private val shoottopflower = poseFactory.of(34.0, 130.0, 90.0)
    private val shoottopflowerControl1 = poseFactory.of(34.0, 100.0, 0.0)
    private val intakesideflowerpart1 = poseFactory.of(34.0, 47.8, 90.0)
    private val intakesideflowerpart2 = poseFactory.of(14.0, 47.8, 180.0)
    private val shootsideflowerpart1 = poseFactory.of(30.0, 47.8, 180.0)
    private val shootsideflowerpart2 = poseFactory.of(34.0, 130.0, 90.0)
    private val park = poseFactory.of(10.0, 115.0, 90.0)
    private val parkControl1 = poseFactory.of(10.0, 130.0, 0.0)

    private fun intakecorner(): Path = Paths.line(start, intakecorner).constant(intakecorner)
        .with(Constants.foresightConfig.maxPathSpeed.at(0.5))

    private fun shootcornernear(): Path = Paths.line(intakecorner, shootcornernear).constant(shootcornernear)

    private fun shootcornerfar(): Path = Paths.line(shootcornernear, shootcornerfar).heading(
        Interpolator.piecewise()
            .until(0.1, Interpolator.constant(shootcornerfarSegment1Heading))
            .until(0.4, Interpolator.linear(shootcornerfarSegment2Start, shootcornerfarSegment2End))
            .until(1.0, Interpolator.constant(shootcornerfarSegment3Heading))
    )

    private fun intaketopflower(): Path =
        Paths.curve(shootcornerfar, intaketopflowerControl1, intaketopflower).constant(intaketopflower)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.7))

    private fun shoottopflower(): Path =
        Paths.curve(intaketopflower, shoottopflowerControl1, shoottopflower).constant(shoottopflower)

    private fun intakesideflowerpart1(): Path =
        Paths.line(shoottopflower, intakesideflowerpart1).constant(intakesideflowerpart1)

    private fun intakesideflowerpart2(): Path =
        Paths.line(intakesideflowerpart1, intakesideflowerpart2).constant(intakesideflowerpart2)
            .with(Constants.foresightConfig.maxPathSpeed.at(0.7))

    private fun shootsideflowerpart1(): Path =
        Paths.line(intakesideflowerpart2, shootsideflowerpart1).constant(shootsideflowerpart1)

    private fun shootsideflowerpart2(): Path =
        Paths.line(shootsideflowerpart1, shootsideflowerpart2).constant(shootsideflowerpart2)

    private fun park(): Path = Paths.curve(shootsideflowerpart2, parkControl1, park).constant(park)
}
