package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.math.Pose

object CurrAlliance {
    private var goalOverride: Pose? = null
    var useGoal1: Boolean = true
        private set
    var cur: Alliance = RedAlliance
        set(value) {
            field = value
            SwitchGoal(if (useGoal1) 1 else 2)
        }
    // Resolve the selected goal from the active alliance's source-defined coordinates.
    // Explicit temporary overrides remain available to standalone tests.
    var CurrGoal: Pose
        get() = goalOverride ?: if (useGoal1) cur.GoalPose1 else cur.GoalPose2
        set(value) { goalOverride = value }

    fun Reset() {
        cur = RedAlliance
        SwitchGoal(1)
    }

    fun SwitchGoal(goalNum: Int) {
        when (goalNum) {
            1 -> {
                useGoal1 = true
                goalOverride = null
            }
            2 -> {
                useGoal1 = false
                goalOverride = null
            }
        }
    }

    fun ToggleGoal() {
        SwitchGoal(if (useGoal1) 2 else 1)
    }

    /** Use measured Y, before applying shooting velocity lead. */
    fun SelectGoalForY(robotY: Double) {
        if (!robotY.isFinite()) return
        val selectGoal1 = robotY <= Alliance.GOAL_MIRROR_Y_IN
        if (useGoal1 != selectGoal1 || goalOverride != null) {
            SwitchGoal(if (selectGoal1) 1 else 2)
        }
    }
}
