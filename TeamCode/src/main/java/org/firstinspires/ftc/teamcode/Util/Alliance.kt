package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.Subsystem.Turret.Turret.goal

open class Alliance(goal1X: Double, goal1Y: Double, goal2X: Double, goal2Y: Double) {
    val GoalPose1 = Pose(goal1X, goal1Y)
    val GoalPose2 = Pose(goal2X, goal2Y)
}
object RedAlliance: Alliance(3.0, 3.0, 2.0, 3.0)
object BlueAlliance: Alliance(-2.0, -2.0, -5.0, -5.0)