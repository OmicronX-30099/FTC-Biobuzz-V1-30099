package org.firstinspires.ftc.teamcode.experimental.physics;

import org.junit.Test;
import static org.junit.Assert.*;

public class PhysicsVelocityTrimTest {
    @Test
    public void affineAdjustmentUsesOneIncrementPerPressAndRetainsOriginalLutValue() {
        PhysicsVelocityTrim trim = new PhysicsVelocityTrim();
        assertEquals(1500, trim.apply(1500), 0);
        trim.update(false, true, true, false);
        assertEquals(1.01, trim.getMultiplier(), 0);
        assertEquals(20, trim.getOffsetTPS(), 0);
        assertEquals(1535, trim.apply(1500), 1e-9);
        trim.update(false, true, true, false);
        assertEquals(1535, trim.apply(1500), 1e-9);
        trim.update(false, false, false, false);
        trim.update(true, false, false, true);
        assertEquals(1, trim.getMultiplier(), 0);
        assertEquals(0, trim.getOffsetTPS(), 0);
        assertEquals(1500, trim.apply(1500), 0);
    }

    @Test
    public void resetAndSeedSuppressHeldButtonsUntilReleased() {
        PhysicsVelocityTrim trim = new PhysicsVelocityTrim();
        trim.seedButtons(true, false, false, true);
        trim.update(true, false, false, true);
        assertEquals(1, trim.getMultiplier(), 0);
        assertEquals(0, trim.getOffsetTPS(), 0);
        trim.update(false, false, false, false);
        trim.update(true, false, false, true);
        assertEquals(.99, trim.getMultiplier(), 0);
        assertEquals(-20, trim.getOffsetTPS(), 0);
        trim.reset(true, false, false, true);
        trim.update(true, false, false, true);
        assertEquals(1, trim.getMultiplier(), 0);
        assertEquals(0, trim.getOffsetTPS(), 0);
    }

    @Test
    public void simultaneousOpposingPressesCancelAndNegativeOutputCannotReverseShooter() {
        PhysicsVelocityTrim trim = new PhysicsVelocityTrim();
        trim.update(true, true, true, true);
        assertEquals(1, trim.getMultiplier(), 0);
        assertEquals(0, trim.getOffsetTPS(), 0);
        for (int i = 0; i < 110; i++) {
            trim.update(false, false, false, false);
            trim.update(true, false, false, true);
        }
        assertEquals(0, trim.getMultiplier(), 0);
        assertEquals(0, trim.apply(1500), 0);
        trim.reset(false, false, false, false);
        trim.update(false, false, true, false);
        assertEquals(0, trim.apply(Double.NaN), 0);
        assertEquals(0, trim.apply(Double.POSITIVE_INFINITY), 0);
        assertEquals(0, trim.apply(-1), 0);
    }
}
