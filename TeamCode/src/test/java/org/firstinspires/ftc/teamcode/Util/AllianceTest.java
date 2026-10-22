package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.control.TurretAiming;
import org.junit.Test;

import static org.junit.Assert.*;

public class AllianceTest {
    @Test
    public void selectsGoalFromActualYWithGoalOneAtTheBoundary() {
        CurrAlliance current = CurrAlliance.INSTANCE;
        Alliance savedAlliance = current.getCur();
        boolean savedUseGoal1 = current.getUseGoal1();
        Pose savedGoal = current.getCurrGoal();
        try {
            current.Reset();
            current.SelectGoalForY(70.750001);
            assertFalse(current.getUseGoal1());
            assertSame(RedAlliance.INSTANCE.getGoalPose2(), current.getCurrGoal());
            current.SelectGoalForY(Double.NaN);
            assertFalse(current.getUseGoal1());
            current.SelectGoalForY(70.75);
            assertTrue(current.getUseGoal1());
            assertSame(RedAlliance.INSTANCE.getGoalPose1(), current.getCurrGoal());
            current.SelectGoalForY(0);
            assertTrue(current.getUseGoal1());
        } finally {
            current.setCur(savedAlliance);
            current.SwitchGoal(savedUseGoal1 ? 1 : 2);
            current.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void dashboardGoalEditsApplyLiveAndSurviveOpModeReset() {
        CurrAlliance current = CurrAlliance.INSTANCE;
        Alliance savedAlliance = current.getCur();
        boolean savedUseGoal1 = current.getUseGoal1();
        Pose savedGoal = current.getCurrGoal();
        double x1 = RedAlliance.GOAL_1_X, y1 = RedAlliance.GOAL_1_Y;
        double x2 = RedAlliance.GOAL_2_X, y2 = RedAlliance.GOAL_2_Y;
        try {
            current.Reset();
            double originalAim = aim(current.getCurrGoal());
            RedAlliance.GOAL_1_X = 30;
            RedAlliance.GOAL_1_Y = 60;
            assertEquals(30, current.getCurrGoal().x(), 0);
            assertEquals(60, current.getCurrGoal().y(), 0);
            assertNotEquals(originalAim, aim(current.getCurrGoal()), 1e-9);
            current.Reset();
            assertEquals(30, current.getCurrGoal().x(), 0);
            assertEquals(60, current.getCurrGoal().y(), 0);

            current.ToggleGoal();
            RedAlliance.GOAL_2_X = 40;
            RedAlliance.GOAL_2_Y = 95;
            assertFalse(current.getUseGoal1());
            assertEquals(40, current.getCurrGoal().x(), 0);
            assertEquals(95, current.getCurrGoal().y(), 0);
            RedAlliance.GOAL_1_X = 35;
            assertEquals(40, current.getCurrGoal().x(), 0);
            current.ToggleGoal();
            assertEquals(35, current.getCurrGoal().x(), 0);
        } finally {
            RedAlliance.GOAL_1_X = x1;
            RedAlliance.GOAL_1_Y = y1;
            RedAlliance.GOAL_2_X = x2;
            RedAlliance.GOAL_2_Y = y2;
            current.setCur(savedAlliance);
            current.SwitchGoal(savedUseGoal1 ? 1 : 2);
            current.setCurrGoal(savedGoal);
        }
    }

    @Test
    public void togglingChangesTheAimingTargetAndResetRestoresRedGoalOne() {
        CurrAlliance current = CurrAlliance.INSTANCE;
        Alliance savedAlliance = current.getCur();
        boolean savedUseGoal1 = current.getUseGoal1();
        Pose savedGoal = current.getCurrGoal();
        try {
            current.Reset();
            Pose goalOne = current.getCurrGoal();
            assertSame(RedAlliance.INSTANCE.getGoalPose1(), goalOne);
            double firstAim = aim(goalOne);

            current.ToggleGoal();
            assertFalse(current.getUseGoal1());
            Pose goalTwo = current.getCurrGoal();
            assertSame(RedAlliance.INSTANCE.getGoalPose2(), goalTwo);
            assertEquals(goalOne.x(), goalTwo.x(), 1e-9);
            assertEquals(70.75, (goalOne.y() + goalTwo.y()) / 2, 1e-9);
            assertTrue(aim(goalTwo) < firstAim);

            current.ToggleGoal();
            assertTrue(current.getUseGoal1());
            assertSame(goalOne, current.getCurrGoal());

            // Reset must also recover from a different alliance and selected goal.
            current.setCur(new Alliance(1, 2, 3, 4));
            current.SwitchGoal(2);
            current.Reset();
            assertSame(RedAlliance.INSTANCE, current.getCur());
            assertTrue(current.getUseGoal1());
            assertSame(RedAlliance.INSTANCE.getGoalPose1(), current.getCurrGoal());
        } finally {
            current.setCur(savedAlliance);
            current.SwitchGoal(savedUseGoal1 ? 1 : 2);
            current.setCurrGoal(savedGoal);
        }
    }

    private static double aim(Pose goal) {
        return TurretAiming.targetDegrees(8.2, 7.3, Math.PI / 2,
                goal.x(), goal.y(), 0, -230, 270, true);
    }
}
