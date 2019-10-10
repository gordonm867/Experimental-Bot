package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Circle;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.CircleCircleIntersection;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Vector2;

public class Odometry implements Subsystem {

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

    private State state = State.ON;

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
        thismetry.setState(State.ON);
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
        return update && state == State.ON;
    }

    public double getUpdateTime() {
        return dTime / (1000 * updates);
    }

    public void update() {
        if(update && state == State.ON) {
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
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot) {
        update();
    }

    public void setState(State newState) {
        this.state = newState;
    }
}
