package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;

public class BallOccupancyTest {
    private final BallOccupancy balls = new BallOccupancy();
    private boolean sample(long ms, int mask) {
        for (int i = 0; i < 4; i++) balls.sample(i, (mask & (1 << i)) != 0,
                ms * 1_000_000L, 40_000_000L);
        return balls.finishSample();
    }

    @Test public void rumblesOnceWhenFourSlotsFillIncludingAtStartup() {
        assertFalse(sample(0, 15));
        assertFalse(sample(39, 15));
        assertTrue(sample(40, 15));
        assertEquals(4, balls.getCount());
        assertFalse(sample(200, 15));
    }

    @Test public void briefClearDoesNotRetriggerButConfirmedClearRearms() {
        sample(0, 15);
        assertTrue(sample(40, 15));
        sample(50, 7);
        assertFalse(sample(60, 15));
        assertFalse(sample(100, 15));
        sample(110, 7);
        assertFalse(sample(150, 7));
        assertEquals(3, balls.getCount());
        sample(160, 15);
        assertTrue(sample(200, 15));
    }

    @Test public void countsOccupiedSlotsWithoutCountingPassages() {
        sample(0, 5);
        assertFalse(sample(40, 5));
        assertEquals(2, balls.getCount());
        sample(50, 0);
        sample(90, 0);
        assertEquals(0, balls.getCount());
    }
}
