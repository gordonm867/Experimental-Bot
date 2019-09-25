package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.GOFException;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Circle;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.CircleCircleIntersection;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Line;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Vector2;

public class Odometry {

    private static TrashHardware robot;
    private static double lastAngle;
    private double lastXPos = Globals.START_X;
    private double lastYPos = Globals.START_Y;
    private static double angle;
    private double x = Globals.START_X;
    private double y = Globals.START_Y;
    private double lastTime = System.currentTimeMillis();
    private double dTime = 0;
    private int updates;
    private boolean update = true;
    private static Odometry thismetry = null;
    private static double angleOffset = 0;

    public static Odometry getInstance(TrashHardware myRobot) {
        robot = myRobot;
        if(thismetry == null) {
            lastAngle = robot.getAngle();
            angle = robot.getAngle();
            thismetry = new Odometry();
            robot.resetOmnis();
        }
        else {
            angleOffset = lastAngle - Globals.START_THETA;
        }
        return thismetry;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return robot.getAngle() + angleOffset;
    }

    public Point getPoint() {
        return new Point(x, y);
    }

    public boolean isUpdating() {
        return update;
    }

    public double getUpdateTime() {
        return dTime / (1000 * updates);
    }

    public void update() {
        if(update) {
            updates++;
            angle = robot.getAngle() + angleOffset;
            double dTheta = angle - lastAngle;
            if(dTheta >= 300) {
                dTheta -= 360;
            }
            if(dTheta <= -300) {
                dTheta += 360;
            }
            lastAngle = angle;
            double xDist = (robot.getHOmniPos() - lastXPos) * Globals.OMNI_FEET_PER_TICK;
            double yDist = (robot.getVOmniPos() - lastYPos) * Globals.OMNI_FEET_PER_TICK;
            double displacement = Math.hypot(xDist, yDist);
            angle += Math.toDegrees(Math.atan2(yDist, xDist)) - (dTheta / 2);
            // double arcAngle = Math.toRadians(angle + Math.toDegrees(Math.atan2(yDist, xDist)));
            if (dTheta != 0 && displacement > 0) {
                /*
                CircleCircleIntersection intersection = new CircleCircleIntersection(new Circle(new Vector2(x, y), displacement), new Circle(new Vector2(0, 0), (Math.sqrt(Math.pow(displacement, 2) / (4 * Math.pow(Math.sin(0.5 * dTheta), 2))))));
                double direction = Math.signum(displacement * Math.cos(Math.toRadians(angle)));
                Vector2[] points = intersection.getIntersectionPoints();
                */
                double radius = Math.abs((((360.0 / dTheta) * displacement) / (2.0 * Math.PI)));
                Circle myCircle = new Circle(new Point(x, y), (((2 * radius * Math.sin(Math.toRadians(dTheta) / 2)))));
                Line myLine = new Line(new Point(x, y), new Point(x + 1, y + Math.tan(Math.toRadians(angle))));
                Point[] points;
                try {
                    if(Math.abs(Math.cos(Math.toRadians(angle))) <= 0.05) {
                        throw new GOFException("You have engaged in a movement incompatible with this robot.  If you do that again, prepare to die.");
                    }
                    Object[] objs = Functions.infiniteLineCircleIntersection(myCircle, myLine).toArray();
                    points = new Point[objs == null ? 0 : objs.length];
                    int co = 0;
                    if(objs != null) {
                        for(Object obj : objs) {
                            if(obj instanceof Point) {
                                points[co] = (Point)obj;
                            }
                            co++;
                        }
                    }
                }
                catch(Exception p_exception) {
                    points = new Point[0];
                }
                Point approxPoint = new Point(x + (displacement * Math.cos(Math.toRadians(angle))), y + (displacement * Math.sin(Math.toRadians(angle))));
                if(points.length == 0) {
                    x += displacement * Math.cos(Math.toRadians(angle));
                    y += displacement * Math.sin(Math.toRadians(angle));
                }
                else if(points.length == 1) {
                    x = points[0].getX();
                    y = points[0].getY();
                }
                else {
                    Point testPoint = points[0];
                    Point testPoint2 = points[1];
                    int dist1 = testPoint.distance(approxPoint);
                    int dist2 = testPoint2.distance(approxPoint);
                    if(Math.abs(dist1) < Math.abs(dist2)) {
                        x = points[0].getX();
                        y = points[0].getY();
                    }
                    else {
                        x = points[1].getX();
                        y = points[1].getY();
                    }
                }
            }
            else {
                x += displacement * Math.cos(Math.toRadians(angle));
                y += displacement * Math.sin(Math.toRadians(angle));
            }
            lastXPos = robot.getHOmniPos();
            lastYPos = robot.getVOmniPos();
            dTime += System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();
        }

        /*
        if(update) {
            updates++;
            angle = robot.getAngle() + angleOffset;
            double dTheta = angle - lastAngle;
            if(dTheta >= 300) {
                dTheta -= 360;
            }
            if(dTheta <= -300) {
                dTheta += 360;
            }
            lastAngle = angle;
            double xDist = (robot.getHOmniPos() - lastXPos) * Globals.OMNI_FEET_PER_TICK;
            double yDist = (robot.getVOmniPos() - lastYPos) * Globals.OMNI_FEET_PER_TICK;
            double displacement = Math.hypot(xDist, yDist);
            angle += Math.toDegrees(Math.atan2(yDist, xDist));
            // double arcAngle = Math.toRadians(angle + Math.toDegrees(Math.atan2(yDist, xDist)));
            if (dTheta != 0 && displacement > 0) {
                CircleCircleIntersection intersection = new CircleCircleIntersection(new Circle(new Vector2(x, y), displacement), new Circle(new Vector2(0, 0), (Math.sqrt(Math.pow(displacement, 2) / (4 * Math.pow(Math.sin(0.5 * dTheta), 2))))));
                double direction = Math.signum(displacement * Math.cos(Math.toRadians(angle)));
                Vector2[] points = intersection.getIntersectionPoints();
                if(points.length == 0) {
                    x += displacement * Math.cos(Math.toRadians(angle));
                    y += displacement * Math.sin(Math.toRadians(angle));
                }
                else if(points.length == 1) {
                    x = points[0].x;
                    y = points[0].y;
                }
                else if (direction > 0) {
                    if(points[0].x >= x) {
                        x = points[0].x;
                        y = points[0].y;
                    }
                    else {
                        x = points[1].x;
                        y = points[1].y;
                    }
                }
                else if (direction < 0) {
                    if(points[0].x >= x) {
                        x = points[1].x;
                        y = points[1].y;
                    }
                    else {
                        x = points[0].x;
                        y = points[0].y;
                    }
                }
                else {
                    direction = Math.signum(displacement * Math.sin(Math.toRadians(angle)));
                    if(direction > 0) {
                        if(points[0].y >= y) {
                            x = points[0].x;
                            y = points[0].y;
                        }
                        else {
                            x = points[1].x;
                            y = points[1].y;
                        }
                    }
                    else if(direction < 0) {
                        if(points[0].y <= y) {
                            x = points[0].x;
                            y = points[0].y;
                        }
                        else {
                            x = points[1].x;
                            y = points[1].y;
                        }
                    }
                    else {
                        x = x;
                        y = y;
                    }
                }
            }
            else {
                x += displacement * Math.cos(Math.toRadians(angle));
                y += displacement * Math.sin(Math.toRadians(angle));
            }
            lastXPos = robot.getHOmniPos();
            lastYPos = robot.getVOmniPos();
            dTime += System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();
        }

        */
    }
}