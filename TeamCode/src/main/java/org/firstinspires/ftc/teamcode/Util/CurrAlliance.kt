package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.math.Pose

object CurrAlliance {
    var cur = Alliance(0.0, 0.0, 0.0, 0.0)
    var useGoal1: Boolean = false
    var CurrGoal = Pose(0.0, 0.0)
    fun Reset() {
        cur = Alliance(0.0, 0.0, 0.0, 0.0)
        useGoal1 = false
    }
    fun SwitchGoal(goalNum: Int) {
        if (goalNum == 1) {CurrGoal = cur.GoalPose1}
        else if (goalNum == 2) {CurrGoal = cur.GoalPose2}
    }
}