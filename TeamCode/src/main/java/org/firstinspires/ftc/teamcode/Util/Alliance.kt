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
    private var aim1 = Pose(goal1X, goal1Y)
    private var aim2 = Pose(goal2X, goal2Y)
    open val AimPose1: Pose get() = aim1
    open val AimPose2: Pose get() = aim2

    protected fun aim1At(x: Double, y: Double): Pose {
        if (aim1.x() != x || aim1.y() != y) aim1 = Pose(x, y)
        return aim1
    }

    protected fun aim2At(x: Double, y: Double): Pose {
        if (aim2.x() != x || aim2.y() != y) aim2 = Pose(x, y)
        return aim2
    }

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

object RedAlliance: Alliance(58.5, 53.0) {
    // Turret-only targets, independent of the goal coordinates used for shot distance.
    @JvmField @Volatile var AIM_1_X = 58.5
    @JvmField @Volatile var AIM_1_Y = 58.0
    @JvmField @Volatile var AIM_2_X = 58.5
    @JvmField @Volatile var AIM_2_Y = 141.5-58.0

    override val AimPose1: Pose get() = aim1At(AIM_1_X, AIM_1_Y)
    override val AimPose2: Pose get() = aim2At(AIM_2_X, AIM_2_Y)
    @JvmField @Volatile var GOAL_1_X = 58.5
    @JvmField @Volatile var GOAL_1_Y = 53.0
    @JvmField @Volatile var GOAL_2_X = 58.5
    @JvmField @Volatile var GOAL_2_Y = 141.5-53.0

    override val GoalPose1: Pose get() = goal1At(GOAL_1_X, GOAL_1_Y)
    override val GoalPose2: Pose get() = goal2At(GOAL_2_X, GOAL_2_Y)
}

// Defaults are mirrored across y = 70.75; edit each goal's coordinates in source.
object BlueAlliance: Alliance(141.5-58.5, 53.0) {
    @JvmField @Volatile var AIM_1_X = 141.5-58.5
    @JvmField @Volatile var AIM_1_Y = 53.0
    @JvmField @Volatile var AIM_2_X = 141.5-58.5
    @JvmField @Volatile var AIM_2_Y = 141.5-53.0

    override val AimPose1: Pose get() = aim1At(AIM_1_X, AIM_1_Y)
    override val AimPose2: Pose get() = aim2At(AIM_2_X, AIM_2_Y)
    @JvmField @Volatile var GOAL_1_X = 141.5-58.5
    @JvmField @Volatile var GOAL_1_Y = 53.0
    @JvmField @Volatile var GOAL_2_X = 141.5-58.5
    @JvmField @Volatile var GOAL_2_Y = 141.5-53.0

    override val GoalPose1: Pose get() = goal1At(GOAL_1_X, GOAL_1_Y)
    override val GoalPose2: Pose get() = goal2At(GOAL_2_X, GOAL_2_Y)
}
