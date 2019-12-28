package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

public class RGBA {

    private double r;
    private double g;
    private double b;
    private double a;

    public RGBA(double r, double g, double b, double a) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.a = a;
    }

    public double r() {
        return r;
    }

    public double g() {
        return g;
    }

    public double b() {
        return b;
    }

    public double a() {
        return a;
    }

    public String toString() {
        return "RED: " + r + ", GREEN: " + g + ", BLUE: " + b + ", ALPHA: " + a;
    }
}
