package org.firstinspires.ftc.teamcode.Auto

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.ivy.pedro.PedroCommands.hold
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.Subsystem.Breakbeams
import org.firstinspires.ftc.teamcode.Subsystem.Flowers
import org.firstinspires.ftc.teamcode.Subsystem.Rollers
import org.firstinspires.ftc.teamcode.Subsystem.Shooter
import org.firstinspires.ftc.teamcode.Subsystem.Turret.ShotCalibration
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Encoders
import org.firstinspires.ftc.teamcode.OpModeStorage
import org.firstinspires.ftc.teamcode.Util.CurrAlliance
import org.firstinspires.ftc.teamcode.Util.ServoPositions
import org.firstinspires.ftc.teamcode.control.DistanceShotInterpolator
import org.firstinspires.ftc.teamcode.pedro.Constants
import kotlin.math.hypot

@Autonomous(name = "Red Auto Manual Update Test", group = "Red")
class RedAutoManualTest: OpMode()  {
    private lateinit var follower: Follower
    private lateinit var fullPathRed: Command
    private lateinit var hubs: List<LynxModule>
    private lateinit var originalHubCachingModes: List<LynxModule.BulkCachingMode>
    private var shooterInitialized = false
    private var rollersInitialized = false
    private var autoShooting = false
    private var autoIntaking = false
    private var mechanismUpdatesEnabled = false
    private var shotTargetValid = false
    private var autoStartNanos = 0L
    private var turretParkStarted = false

    override fun init() {
        Scheduler.reset()
        autoShooting = false
        autoIntaking = false
        mechanismUpdatesEnabled = false
        autoStartNanos = 0L
        turretParkStarted = false
        follower = Constants.create(hardwareMap)
        follower.setPose(start)
        CurrAlliance.Reset()
        Shooter.init(hardwareMap)
        Shooter.toggleMode(Shooter.Mode.AUTO)
        Shooter.autoFlywheel = 0.0
        Shooter.autoHood = ServoPositions.HOOD_DEFAULT
        shotTargetValid = false
        shooterInitialized = true
        Rollers.init(hardwareMap)
        rollersInitialized = true
        Flowers.init(hardwareMap)
        Breakbeams.init(hardwareMap)
        hubs = hardwareMap.getAll(LynxModule::class.java)
        originalHubCachingModes = hubs.map { it.bulkCachingMode }
        hubs.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        fullPathRed = sequential(
            waitMs(1500.0),
            shootForOneSecond(),
            instant { setShotTarget(openPollenIntake) },
            parallel(
                setIntaking(true),
                follow(follower, path1())
            ),
            follow(follower, path2()),
            setIntaking(false),
            follow(follower, Paths.path(path3(), path4())),
            parallel(instant { Flowers.setDown(true) }, follow(follower, openPollenIntake())),
            waitMs(500.0),
            shootForOneSecond(),
            setIntaking(true),
            follow(follower, path6()),
            hold(follower, point6),
            waitMs(1000.0),
            setIntaking(false),
            follow(follower, path7()),
            waitMs(500.0),
            shootForOneSecond(),
            follow(follower, path8()),
            follow(follower, path9()),
            follow(follower, path10()),
            parallel(setIntaking(true), follow(follower, openPollenIntake2())),
            hold(follower, openPollenIntake2),
            waitMs(1000.0),
            setIntaking(false),
            follow(follower, path12()),
            follow(follower, path13()),
            follow(follower, path14()),
            waitMs(500.0),
            shootForOneSecond(),
            follow(follower, path15()),
            finishAuto()
        )
    }

    override fun start() {
        hubs.forEach { it.clearBulkCache() }
        autoStartNanos = System.nanoTime()
        setShotTarget(start)
        Shooter.start()
        mechanismUpdatesEnabled = true
        Shooter.update(follower.pose())
        schedule(fullPathRed)
        super.start()
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
        if (mechanismUpdatesEnabled) {
            Shooter.update(pose)
            Shooter.setGate(autoShooting)
            Rollers.update(if (autoIntaking) 1.0f else 0.0f, 0.0f, Shooter.gateOpen,
                gateOpenTargetTPS = 1200.0)
        } else if (turretParkStarted) {
            Turret.updateToDegrees(0.0)
        }

        telemetry.addData("x", pose.x())
        telemetry.addData("y", pose.y())
        telemetry.addData("heading", pose.heading())
        telemetry.addData("Shooting", autoShooting)
        telemetry.addData("Intaking", autoIntaking)
        telemetry.addData("Turret target / angle / power", "%.1f / %.1f / %.2f",
            Turret.targetDegrees, Encoders.trueDegrees, Turret.motorPower)
        telemetry.addData("Flower intake", if (Flowers.isDown) "Down" else "Up")
        telemetry.addData("Balls", Breakbeams.ballCount)
        telemetry.addData("Shooter velocity (ticks/s)", "%.1f", Shooter.shooterVelocity)
        telemetry.addData("Intake target velocity (ticks/s)", "%.1f", Rollers.targetVelocityTPS)

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint())
            telemetry.addData("Path number", follower.pathIndex())
        }

        telemetry.update()
    }

    override fun stop() {
        if (::follower.isInitialized) OpModeStorage.autonomousEndPose = follower.pose()
        Scheduler.reset()
        autoShooting = false
        autoIntaking = false
        mechanismUpdatesEnabled = false
        try {
            if (shooterInitialized) Shooter.stop()
        } finally {
            try {
                if (rollersInitialized) Rollers.stop()
            } finally {
                if (::hubs.isInitialized && ::originalHubCachingModes.isInitialized) {
                    hubs.forEachIndexed { i, hub -> hub.bulkCachingMode = originalHubCachingModes[i] }
                }
            }
        }
    }

    private fun shootForOneSecond(): Command = sequential(
        instant { autoShooting = shotTargetValid },
        waitMs(1000.0),
        instant { autoShooting = false }
    )

    private fun setShotTarget(shootingPoint: Pose) {
        CurrAlliance.SelectGoalForY(shootingPoint.y())
        val goal = CurrAlliance.CurrGoal
        val distance = hypot(goal.x() - shootingPoint.x(), goal.y() - shootingPoint.y())
        val shot = DistanceShotInterpolator.sample(
            distance,
            ShotCalibration.DISTANCE_INCHES,
            ShotCalibration.VELOCITY_TPS,
            ShotCalibration.HOOD_POSITIONS
        )
        if (shot == null) {
            shotTargetValid = false
            Shooter.autoFlywheel = 0.0
            Shooter.autoHood = ServoPositions.HOOD_DEFAULT
        } else {
            shotTargetValid = true
            Shooter.autoFlywheel = shot.velocityTPS
            Shooter.autoHood = shot.hoodPosition
        }
    }


    private fun setIntaking(active: Boolean): Command = instant {
        autoIntaking = active
    }

    private fun finishAuto(): Command = instant {
        autoShooting = false
        autoIntaking = false
        Shooter.stop()
        if (turretParkStarted) {
            Turret.parkAtZero()
        }
        Rollers.stop()
        mechanismUpdatesEnabled = false
    }
    private val poseFactory = PoseFactory.degrees()

    private val start = poseFactory.of(54.4, 7.3, 180.0)
    private val path1 = poseFactory.of(10.5, 7.3, 180.0)
    private val point2Start = poseFactory.of(10.5, 7.3, 180.0)
    private val point2 = poseFactory.of(24.0, 15.0, 90.0)
    private val point3 = poseFactory.of(24.0, 80.0, 90.0)
    private val point4 = poseFactory.of(48.0, 115.0, 36.1373)
    private val point4Control1 = poseFactory.of(24.3341, 97.8072, 0.0)
    private val openPollenIntakeStart = poseFactory.of(48.0, 115.0, 35.0)
    private val openPollenIntake = poseFactory.of(64.0, 127.0, 90.0)
    private val point6 = poseFactory.of(48.3, 126.8, 90.0)
    private val point6Control1 = poseFactory.of(50.0, 100.0, 0.0)
    private val point7 = poseFactory.of(64.0, 127.0, 90.0)
    private val point7Control1 = poseFactory.of(50.0, 100.0, 0.0)
    private val point8 = poseFactory.of(30.0, 90.0, 90.0)
    private val point9 = poseFactory.of(27.0, 61.5, 90.0)
    private val point10Start = poseFactory.of(27.0, 61.5, 90.0)
    private val point10 = poseFactory.of(27.0, 47.5, 180.0)
    private val openPollenIntake2 = poseFactory.of(15.2, 47.8, 180.0)
    private val point12Start = poseFactory.of(15.2, 47.8, 180.0)
    private val point12 = poseFactory.of(27.3655, 57.0258, 90.0)
    private val point13 = poseFactory.of(28.0, 111.0, 90.0)
    private val point14 = poseFactory.of(64.0, 127.0, 90.0)
    private val path15 = poseFactory.of(10.0, 105.0, 90.0)

    fun path1(): Path = Paths.line(start, path1).constant(path1)
        .with(Constants.foresightConfig.maxPathSpeed.at(0.5))

    fun path2(): Path = Paths.line(point2Start, point2).linear(point2Start, point2)

    fun path3(): Path = Paths.line(point2, point3).constant(point3)

    fun path4(): Path = Paths.curve(point3, point4Control1, point4).tangent()

    fun openPollenIntake(): Path = Paths.line(openPollenIntakeStart, openPollenIntake).linear(openPollenIntakeStart, openPollenIntake)

    fun path6(): Path = Paths.curve(openPollenIntake, point6Control1, point6).constant(point6)
        .with(Constants.foresightConfig.maxPathSpeed.at(0.7))

    fun path7(): Path = Paths.curve(point6, point7Control1, point7).constant(point7)

    fun path8(): Path = Paths.line(point7, point8).constant(point8)

    fun path9(): Path = Paths.line(point8, point9).constant(point9)

    fun path10(): Path = Paths.line(point10Start, point10).linear(point10Start, point10)

    fun openPollenIntake2(): Path = Paths.line(point10, openPollenIntake2).constant(openPollenIntake2)
        .with(Constants.foresightConfig.maxPathSpeed.at(0.7))

    fun path12(): Path = Paths.line(point12Start, point12).linear(point12Start, point12)

    fun path13(): Path = Paths.line(point12, point13).constant(point13)

    fun path14(): Path = Paths.line(point13, point14).constant(point14)

    fun path15(): Path = Paths.line(point14, path15).constant(path15)
    fun followPath(pedroPath: Path): Command {
        return Command.build()
            .setStart() {
                follower.follow(pedroPath)
            }
    }
}
