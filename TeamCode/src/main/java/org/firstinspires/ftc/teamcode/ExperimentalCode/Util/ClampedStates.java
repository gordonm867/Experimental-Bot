package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;

public class ClampedStates extends Point {

    private boolean exact;
    private double delay;
    private boolean locked;
    private double clawPos;
    private boolean clamped;

    public ClampedStates(double x, double y, double theta, boolean locked, double clawPos, boolean clamped, double delay, boolean exact) {
        super(x, y, theta);
        this.locked = locked;
        this.clawPos = clawPos;
        this.clamped = clamped;
        this.delay = delay;
        this.exact = exact;
    }

    public double getDelay() {
        return delay;
    }

    public boolean isExact() {
        return exact;
    }

    public boolean isLocked() {
        return locked;
    }

    public boolean isClamped() {
        return clamped;
    }

    public double getClawPos() {
        return clawPos;
    }

    public void setDelay(double newDelay) {
        this.delay = newDelay;
    }
}
