package org.firstinspires.ftc.teamcode.control;

/** Four occupied slots, with time-based filtering of both blocked and clear edges. */
public final class BallOccupancy {
    private final boolean[] occupied = new boolean[4];
    private final boolean[] candidate = new boolean[4];
    private final long[] changedAt = new long[4];
    private boolean wasFull;
    private int count;

    public void sample(int slot, boolean blocked, long nowNanos, long debounceNanos) {
        if (blocked != candidate[slot]) {
            candidate[slot] = blocked;
            changedAt[slot] = nowNanos;
        }
        if (nowNanos - changedAt[slot] >= debounceNanos) occupied[slot] = blocked;
    }

    /** Call once after sampling all four slots. */
    public boolean finishSample() {
        count = 0;
        for (boolean ball : occupied) if (ball) count++;
        boolean full = count == 4;
        boolean justFull = full && !wasFull;
        wasFull = full;
        return justFull;
    }

    public int getCount() { return count; }
}
