package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.math.Pose;
import org.junit.Test;

import static org.junit.Assert.*;

public class ShootingPosePredictionTest {
    @Test
    public void leadsFieldPositionByHalfASecondWithoutRotatingVelocityOrHeading() {
        Pose measured = new Pose(8.2, 7.3, Math.PI / 2);
        Pose predicted = ShootingPosePrediction.predict(measured, 10, -4);
        assertEquals(13.2, predicted.x(), 1e-9);
        assertEquals(5.3, predicted.y(), 1e-9);
        assertEquals(measured.heading(), predicted.heading(), 0);
        assertEquals(8.2, measured.x(), 0);
        assertEquals(7.3, measured.y(), 0);
        assertSame(measured, ShootingPosePrediction.predict(measured, 0, 0));
    }

    @Test
    public void lateralMotionChangesTheAimFromThePredictedPosition() {
        Pose predicted = ShootingPosePrediction.predict(new Pose(0, 0, 0), 0, 10);
        double target = TurretAiming.targetDegrees(predicted.x(), predicted.y(), predicted.heading(),
                100, 0, 0, -230, 270, true);
        assertEquals(Math.toDegrees(Math.atan2(5, 100)), target, 1e-9);
    }

    @Test
    public void rotationLeadsTurretAgainstRobotTurnInEitherEncoderConvention() {
        for (double omega : new double[] {Math.PI, -Math.PI}) {
            for (boolean clockwisePositive : new boolean[] {true, false}) {
                Pose measured = new Pose(0, 0, 0);
                Pose predicted = ShootingPosePrediction.predict(measured, 0, 0, omega);
                double target = TurretAiming.targetDegrees(predicted.x(), predicted.y(),
                        predicted.heading(), 100, 0, 0, -230, 270, clockwisePositive);
                // At 180 deg/s, the turret anticipates 9 degrees of robot rotation.
                assertEquals(Math.toDegrees(omega) * 0.05 * (clockwisePositive ? 1 : -1),
                        target, 1e-9);
                assertEquals(0, predicted.x(), 0);
                assertEquals(0, predicted.y(), 0);
                assertEquals(0, measured.heading(), 0);
            }
        }
    }

    @Test
    public void combinedMotionPreservesTranslationLeadAndAimsAcrossHeadingWrap() {
        Pose predicted = ShootingPosePrediction.predict(
                new Pose(8.2, 7.3, Math.toRadians(359)), 10, -4, Math.toRadians(40));
        assertEquals(13.2, predicted.x(), 1e-9);
        assertEquals(5.3, predicted.y(), 1e-9);
        double target = TurretAiming.targetDegrees(predicted.x(), predicted.y(), predicted.heading(),
                113.2, 5.3, 0, -230, 270, true);
        assertEquals(1, target, 1e-9);
    }
}
