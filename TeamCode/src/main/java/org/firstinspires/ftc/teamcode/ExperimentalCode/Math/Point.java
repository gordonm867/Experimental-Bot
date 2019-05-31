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
        Point point = new Point(newX, newY);
        return angle(point, unit);
    }

    public double angle(Point newPoint, AngleUnit unit) {
        double angle;
        if(unit == AngleUnit.DEGREES) {
            angle = Math.toDegrees(Math.atan2(newPoint.getY() - y, newPoint.getX() - x));
            while(angle < -180 || angle > 180) {
                angle += 360;
            }
        }
        else {
            angle = Math.atan2(newPoint.getY() - y, newPoint.getX() - x);
            while(angle < -Math.PI || angle > Math.PI) {
                angle += 2 * Math.PI;
            }
        }
        return angle;
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
