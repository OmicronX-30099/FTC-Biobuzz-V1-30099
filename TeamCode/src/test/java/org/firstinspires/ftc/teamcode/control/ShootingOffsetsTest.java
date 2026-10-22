package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShootingOffsetsTest {
    @Test
    public void offsetsAccumulateOnlyOnPressAndOpposingPressesCancel() {
        ShootingOffsets offsets = new ShootingOffsets();
        assertEquals(-2, offsets.update(true, false, true, false), 0);
        assertEquals(20, offsets.getVelocityTPS(), 0);
        assertEquals(0, offsets.update(true, false, true, false), 0);
        assertEquals(20, offsets.getVelocityTPS(), 0);
        offsets.update(false, false, false, false);
        offsets.update(true, false, true, false);
        assertEquals(-4, offsets.getHeadingDegrees(), 0);
        assertEquals(40, offsets.getVelocityTPS(), 0);
        offsets.update(false, false, false, false);
        assertEquals(2, offsets.update(false, true, false, true), 0);
        assertEquals(-2, offsets.getHeadingDegrees(), 0);
        assertEquals(20, offsets.getVelocityTPS(), 0);
        offsets.update(false, false, false, false);
        assertEquals(0, offsets.update(true, true, true, true), 0);
        assertEquals(20, offsets.getVelocityTPS(), 0);
    }

    @Test
    public void resetClearsBothOffsetsAndRequiresHeldButtonsToBeReleased() {
        ShootingOffsets offsets = new ShootingOffsets();
        offsets.update(false, true, false, true);
        offsets.reset(false, true, false, true);
        assertEquals(0, offsets.getHeadingDegrees(), 0);
        assertEquals(0, offsets.getVelocityTPS(), 0);
        assertEquals(0, offsets.update(false, true, false, true), 0);
        assertEquals(0, offsets.getVelocityTPS(), 0);
        offsets.update(false, false, false, false);
        assertEquals(2, offsets.update(false, true, false, true), 0);
        assertEquals(-20, offsets.getVelocityTPS(), 0);
    }
}
