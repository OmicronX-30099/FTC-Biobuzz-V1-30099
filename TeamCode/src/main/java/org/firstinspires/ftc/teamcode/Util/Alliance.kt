package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.math.Pose

open class Alliance(
    goal1X: Double, goal1Y: Double,
    goal2X: Double = goal1X, goal2Y: Double = 2 * GOAL_MIRROR_Y_IN - goal1Y
) {
    companion object {
        const val GOAL_MIRROR_Y_IN = 70.75
    }

    private var goal1 = Pose(goal1X, goal1Y)
    private var goal2 = Pose(goal2X, goal2Y)
    open val GoalPose1: Pose get() = goal1
    open val GoalPose2: Pose get() = goal2

    // Rebuild only when coordinates change, so normal aiming does not allocate each loop.
    protected fun goal1At(x: Double, y: Double): Pose {
        if (goal1.x() != x || goal1.y() != y) goal1 = Pose(x, y)
        return goal1
    }

    protected fun goal2At(x: Double, y: Double): Pose {
        if (goal2.x() != x || goal2.y() != y) goal2 = Pose(x, y)
        return goal2
    }
}

object RedAlliance: Alliance(57.9, 56.0) {
    @JvmField @Volatile var GOAL_1_X = 56.0
    @JvmField @Volatile var GOAL_1_Y = 59.0
    @JvmField @Volatile var GOAL_2_X = 56.0
    @JvmField @Volatile var GOAL_2_Y = 141.5-59.0

    override val GoalPose1: Pose get() = goal1At(GOAL_1_X, GOAL_1_Y)
    override val GoalPose2: Pose get() = goal2At(GOAL_2_X, GOAL_2_Y)
}

// Defaults are mirrored across y = 70.75; edit each goal's coordinates in source.
object BlueAlliance: Alliance(141.5-57.9, 56.0) {
    @JvmField @Volatile var GOAL_1_X = 141.5-56.0
    @JvmField @Volatile var GOAL_1_Y = 59.0
    @JvmField @Volatile var GOAL_2_X = 141.5-56.0
    @JvmField @Volatile var GOAL_2_Y = 141.5-59.0

    override val GoalPose1: Pose get() = goal1At(GOAL_1_X, GOAL_1_Y)
    override val GoalPose2: Pose get() = goal2At(GOAL_2_X, GOAL_2_Y)
}
