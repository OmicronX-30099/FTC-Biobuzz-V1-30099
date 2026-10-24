package org.firstinspires.ftc.teamcode.control;

/** Edge-triggered driver corrections; no hardware access or per-loop allocations. */
public final class ShootingOffsets {
    private boolean leftWasPressed, rightWasPressed, upWasPressed, downWasPressed;
    private double aimOffsetDegrees, velocityTPS;

    public double getAimOffsetDegrees() { return aimOffsetDegrees; }
    public double getVelocityTPS() { return velocityTPS; }

    public void seedButtons(boolean left, boolean right, boolean up, boolean down) {
        leftWasPressed = left;
        rightWasPressed = right;
        upWasPressed = up;
        downWasPressed = down;
    }

    public void reset(boolean left, boolean right, boolean up, boolean down) {
        aimOffsetDegrees = 0;
        velocityTPS = 0;
        seedButtons(left, right, up, down);
    }

    /** Returns only the new aim correction, in degrees. */
    public double update(boolean left, boolean right, boolean up, boolean down) {
        double delta = 0;
        if (left && !leftWasPressed) delta -= 2;
        if (right && !rightWasPressed) delta += 2;
        if (up && !upWasPressed) velocityTPS += 20;
        if (down && !downWasPressed) velocityTPS -= 20;
        aimOffsetDegrees += delta;
        seedButtons(left, right, up, down);
        return delta;
    }
}
