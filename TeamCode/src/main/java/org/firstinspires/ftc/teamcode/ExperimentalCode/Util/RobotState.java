package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;

public class RobotState extends Point {

    private int extend_pos;
    private double intakepower;
    private boolean required;
    private boolean exact;
    private double delay;
    private double boxPos;

    public RobotState(double x, double y, double theta, int extend_pos, double intakepower, boolean required, double boxPos, double delay, boolean exact) {
        super(x, y, theta);
        this.extend_pos = extend_pos;
        this.intakepower = intakepower;
        this.required = required;
        this.delay = delay;
        this.boxPos = boxPos;
        this.exact = exact;
    }

    public int getExtendPos() {
        return extend_pos;
    }

    public double getIntakePower() {
        return intakepower;
    }

    public boolean isRequired() {
        return required;
    }

    public double getDelay() {
        return delay;
    }

    public double getBoxPos() {
        return boxPos;
    }

    public boolean isExact() {
        return exact;
    }

    public void setDelay(double newDelay) {
        delay = newDelay;
    }

    public void setExact(boolean ex) {
        exact = ex;
    }
}
