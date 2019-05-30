package org.firstinspires.ftc.teamcode.ExperimentalCode.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;

public class Point {

    private double x;
    private double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distance(double newX, double newY) {
        return(int)(Math.round((Math.sqrt(Math.pow(x - newX, 2) + Math.pow(y - newY, 2))) / -Globals.DRIVE_FEET_PER_TICK));
    }

    public int distance(Point newPoint) {
        return(int)(Math.round((Math.sqrt(Math.pow(x - newPoint.getX(), 2) + Math.pow(y - newPoint.getY(), 2))) / -Globals.DRIVE_FEET_PER_TICK));
    }

    public double angle(double newX, double newY, AngleUnit unit) {
        if(unit == AngleUnit.DEGREES) {
            return(Math.toDegrees(Math.atan2(newY - y, newX - x)));
        }
        return Math.atan2(newY - y, newX - x);
    }

    public double angle(Point newPoint, AngleUnit unit) {
        if(unit == AngleUnit.DEGREES) {
            return(Math.toDegrees(Math.atan2(newPoint.getY() - y, newPoint.getX() - x)));
        }
        return Math.atan2(newPoint.getY() - y, newPoint.getX() - x);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public String toString() {
        return "(" + getX() + ", " + getY() + ")";
    }
}
