package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Circle;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Line;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Autonomous(name="TrashAuto", group="Trash")
public class TrashAutonomous extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.STOPPED);
    private double angleOffset = 3;
    private double radius = 1.86;
    private double turnCoeff = 0.0055;
    private Point stopped;
    private Point stopTarget;
    private boolean test = false;

    public void runOpMode() throws InterruptedException {

        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);

        /* Select Configuration */
        boolean changed = false;
        ArrayList<String> pathNames = new ArrayList<>();
        pathNames.add("Rover Ruckus Crater Auto Curve");
        pathNames.add("Circle");
        pathNames.add("S Curve");
        pathNames.add("Skystone Loading Auto");
        int pathNum = 0;
        while(!isStopRequested() && !gamepad1.a) {
            telemetry.addData("Selected Path", pathNames.get(pathNum % pathNames.size()));
            telemetry.update();
            if(!changed && gamepad1.b) {
                pathNum++;
                changed = true;
            }
            if(changed && !gamepad1.b) {
                changed = false;
            }
        }
        while(gamepad1.a) {}
        changed = false;
        while(!isStopRequested() && !gamepad1.a) {
            telemetry.addData("Experimental", (test ? "YES" : "NO"));
            telemetry.update();
            if(!changed && gamepad1.b) {
                test = !test;
                changed = true;
            }
        }

        /* Initialize Subsystems */
        if(pathNum % pathNames.size() == 0) {
            Globals.START_X = -2;
            Globals.START_Y = 2;
            Globals.START_THETA = 135;
        }
        else if(pathNum % pathNames.size() == 1) {
            Globals.START_X = 0;
            Globals.START_Y = 0;
            Globals.START_THETA = 0;
        }
        else if(pathNum % pathNames.size() == 2) {
            Globals.START_X = 0;
            Globals.START_Y = 0;
            Globals.START_THETA = 135;
        }
        else if(pathNum % pathNames.size() == 3) {
            Globals.START_X = -5.25;
            Globals.START_Y = -3;
            Globals.START_THETA = 0;
        }
        odometry = Odometry.getInstance(robot);
        robot.enabled = true;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", pathNames.get(pathNum % pathNames.size()));
        telemetry.update();
        waitForStart();
        wheels.setState(Drivetrain.State.DRIVING);

        ArrayList<Line> path = new ArrayList<>();
        if(pathNum % pathNames.size() == 0) {
            turnToPoint(new Point(-4.7, 0));
            path.add(new Line(odometry.getPoint(), new Point(-4.7, 0)));
            path.add(new Line(new Point(-4.7, 0), new Point(-5, -3.5)));
            follow(path);
            robot.setDrivePower(0, 0, 0, 0);
            path.clear();
            path.add(new Line(new Point(-5, -3.5), new Point(-4.7, 0)));
            path.add(new Line(new Point(-4.7, 0), new Point(-2, 2)));
            backFollow(path);
            turn((((-robot.getAngle() + 135) + 180) % 360) - 180);
            double odoUpdate = odometry.getUpdateTime();
            while (opModeIsActive()) {
                robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wheels.update(robot, new Point(Globals.START_X, Globals.START_Y), odometry, 135, AngleUnit.DEGREES);
                odometry.update();
                telemetry.addData("Point", odometry.getPoint());
                telemetry.addData("Odometry update time", odoUpdate);
                telemetry.update();
            }
        }
        else if(pathNum % pathNames.size() == 1) {
            radius = 1;
            turnCoeff = 0.005;
            turnToPoint(new Point(1, 1));
            path.add(new Line(odometry.getPoint(), new Point(1, 1)));
            path.add(new Line(new Point(1, 1), new Point(0, Math.sqrt(2))));
            path.add(new Line(new Point(0, Math.sqrt(2)), new Point(-1, 1)));
            path.add(new Line(new Point(-1, 1), new Point(-Math.sqrt(2), 0)));
            path.add(new Line(new Point(-Math.sqrt(2), 0), new Point(-1, -1)));
            path.add(new Line(new Point(-1, -1), new Point(0, -Math.sqrt(2))));
            path.add(new Line(new Point(0, -Math.sqrt(2)), new Point(1, -1)));
            path.add(new Line(new Point(1, -1), new Point(Math.sqrt(2), 0)));
            path.add(new Line(new Point(Math.sqrt(2), 0), new Point(0, 0)));
            follow(path);
            runToPoint(new Point(0, 0));
            turn((((-robot.getAngle() + 135) + 180) % 360) - 180);
            double odoUpdate = odometry.getUpdateTime();
            while (opModeIsActive()) {
                robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wheels.update(robot, new Point(Globals.START_X, Globals.START_Y), odometry, 0, AngleUnit.DEGREES);
                odometry.update();
                telemetry.addData("Point", odometry.getPoint());
                telemetry.addData("Odometry update time", odoUpdate);
                telemetry.update();
            }
        }
        else if(pathNum % pathNames.size() == 2) {
            stopped = new Point(1.25, 1.25);
            stopTarget = new Point(1.25, 1.25);
            radius = 0.75;
            turnCoeff = 0.0055;
            strafeTurn(-45, angleOffset, new Point(1.25, 1.25));
            double displacement = Math.abs(Math.sqrt(Math.pow(odometry.getX() - 1.25, 2) + Math.pow(odometry.getY() - 1.25, 2)));
            while(opModeIsActive() && Math.abs(displacement) > 0.15) {
                wheels.update(robot, new Point(1.25, 1.25), odometry, Double.NaN, AngleUnit.DEGREES);
            }
            path.add(new Line(odometry.getPoint(), new Point(1.15, 1.74)));
            for(double x = 1.25 - 0.1; x >= -1.25 + 0.1; x -= 0.1) {
                path.add(new Line(new Point(x, 2.5 * Math.sqrt(0.25 - Math.pow((x / 2.5), 2)) + 1.25), new Point(x - 0.1, 2.5 * Math.sqrt(0.25 - Math.pow(((x - 0.1) / 2.5), 2)) + 1.25)));
            }
            for(double x = -1.25; x <= 0 - 0.1; x += 0.1) {
                path.add(new Line(new Point(x, -2.5 * Math.sqrt(0.25 - Math.pow((x / 2.5), 2)) + 1.25), new Point(x + 0.1, -2.5 * Math.sqrt(0.25 - Math.pow(((x + 0.1) / 2.5), 2)) + 1.25)));
            }
            for(double x = 0; x <= 1.25 - 0.1; x += 0.1) {
                path.add(new Line(new Point(x, 2.5 * Math.sqrt(0.25 - Math.pow((x / 2.5), 2)) - 1.25), new Point(x + 0.1, 2.5 * Math.sqrt(0.25 - Math.pow(((x + 0.1) / 2.5), 2)) - 1.25)));
            }
            for(double x = 1.25; x >= -1.25 + 0.1; x -= 0.1) {
                path.add(new Line(new Point(x, -2.5 * Math.sqrt(0.25 - Math.pow((x / 2.5), 2)) - 1.25), new Point(x - 0.1, -2.5 * Math.sqrt(0.25 - Math.pow(((x - 0.1) / 2.5), 2)) - 1.25)));
            }
            if(path.get(0).getYComp() == 0) {
                path.set(0, new Line(path.get(0).getPoint1(), new Point(path.get(0).getPoint2().getX(), path.get(0).getPoint2().getY() + 0.01)));
            }
            for(int x = 1; x < path.size(); x++) {
                if(path.get(x).getYComp() == 0) {
                    path.set(x, new Line(path.get(x).getPoint1(), new Point(path.get(x).getPoint2().getX(), path.get(x).getPoint2().getY() + 0.01)));
                }
                if(!(path.get(x).getPoint1().equals(path.get(x - 1).getPoint2()))) {
                    path.add(x, new Line(path.get(x - 1).getPoint2(), path.get(x).getPoint1()));
                }
            }
            follow(path);
            double odoUpdate = odometry.getUpdateTime();
            while (opModeIsActive()) {
                robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wheels.update(robot, new Point(1.25, 1.25), odometry, Double.NaN, AngleUnit.DEGREES);
                odometry.update();
                telemetry.addData("Point", odometry.getPoint());
                telemetry.addData("Odometry update time", odoUpdate);
                telemetry.addData("Follower",  "Stopped at " + stopped + " with a target of " + stopTarget + ".");
                telemetry.update();
            }
        }
        else if(pathNum % pathNames.size() == 3) {
            radius = 0.75;
            turnCoeff = 0.0055;
            // int skystone = (int)(Math.ceil(Math.random() * 3));
            int skystone = 1;
            if(skystone == 1) {
                turnToPoint(new Point(-4.25, -3.51));
                path.add(new Line(odometry.getPoint(), new Point(-4.25, -3.51)));
                path.add(new Line(new Point(-4.25, -3.51), new Point(-2.95, -3.5)));
                follow(path);
                turn(-robot.getAngle());
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(200);
                path.add(new Line(odometry.getPoint(), new Point(-3.3, -3.5)));
                backFollow(path);
                robot.setDrivePower(0, 0, 0, 0);
                strafeTurn(0, angleOffset, new Point(-3.3, 3));
                strafeTurn(0, angleOffset, new Point(-2.6, 3));
                robot.setDrivePower(0, 0, 0, 0);
                sleep(200);
                path.clear();
                if(System.currentTimeMillis() - time <= 18000 || test) {
                    path.add(new Line(odometry.getPoint(), new Point(-3, 3)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    path.clear();
                    turn(-robot.getAngle());
                    strafeTurn(0, angleOffset, new Point(-2.87, -4.98));
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(-robot.getAngle() - 25.7142857143);
                    sleep(200);
                    turn(-robot.getAngle());
                    path.add(new Line(odometry.getPoint(), new Point(-3, odometry.getPoint().getY() + 0.01)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    strafeTurn(0, angleOffset, new Point(-3.3, 3));
                    strafeTurn(0, angleOffset, new Point(-2.35, 3));
                    path.clear();
                    strafeTurn(-odometry.getAngle(), angleOffset, new Point(-4, 4));
                    strafeTurn(-odometry.getAngle(), angleOffset, new Point(-4.6, 4));
                    path.add(new Line(odometry.getPoint(), new Point(-4.5, 0)));
                }
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(-4.5, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                }
            }
            if(skystone == 2) {
                turnToPoint(new Point(-3, -3.14));
                path.add(new Line(odometry.getPoint(), new Point(-3, -3.14)));
                path.add(new Line(new Point(-3, -3.6), new Point(-2.2, -3.19)));
                follow(path);
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                path.add(new Line(odometry.getPoint(), new Point(-2.35, -2.94)));
                path.add(new Line(new Point(-2.35, -2.94), new Point(-3.5, -1.64)));
                path.add(new Line(new Point(-3.5, -1.64), new Point(-3.647, 2)));
                path.add(new Line(new Point(-3.647, 2), new Point(-2.07, 2.59)));
                backFollow(path);
                robot.setDrivePower(0, 0, 0, 0);
                turn(-robot.getAngle() - 135);
                path.clear();
                if(System.currentTimeMillis() - time <= 18000 || test) {
                    path.add(new Line(odometry.getPoint(), new Point(-3.75, 0)));
                    path.add(new Line(new Point(-3.75, 0), new Point(-2.81, -5.1)));
                    follow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    path.clear();
                    turn(-robot.getAngle());
                    sleep(200);
                    turnToPoint(new Point(-2.8, -5));
                    path.add(new Line(odometry.getPoint(), new Point(-2.8, -5)));
                    path.add(new Line(new Point(-2.8, -5), new Point(-3, 1.75)));
                    path.add(new Line(new Point(-3, 1.75), new Point(-2.07, 2.59)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(-robot.getAngle() - 135);
                    path.clear();
                    strafeTurn(-odometry.getAngle(), angleOffset, new Point(-4, 4));
                    strafeTurn(-odometry.getAngle(), angleOffset, new Point(-4.6, 4));
                    path.add(new Line(odometry.getPoint(), new Point(-4.5, 0)));
                }
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(-4.5, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                }
            }
            if(skystone == 3) {
                turnToPoint(new Point(-4.25, -2.6));
                path.add(new Line(odometry.getPoint(), new Point(-4.45, -2.6)));
                path.add(new Line(new Point(-4.25, -2.6), new Point(-2.95, -2.4)));
                follow(path);
                turn(-robot.getAngle());
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(200);
                path.add(new Line(odometry.getPoint(), new Point(-3.3, -2.6)));
                backFollow(path);
                robot.setDrivePower(0, 0, 0, 0);
                strafeTurn(0, angleOffset, new Point(-3.3, 3));
                strafeTurn(0, angleOffset, new Point(-2.6, 3));
                robot.setDrivePower(0, 0, 0, 0);
                sleep(200);
                path.clear();
                if(System.currentTimeMillis() - time <= 18000 || test) {
                    path.add(new Line(odometry.getPoint(), new Point(-3, 3)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    path.clear();
                    turn(-robot.getAngle());
                    strafeTurn(0, angleOffset, new Point(-3, -4.25));
                    robot.setDrivePower(0, 0, 0, 0);
                    path.add(new Line(odometry.getPoint(), new Point(-2.95, -4.251)));
                    follow(path);
                    sleep(200);
                    path.add(new Line(odometry.getPoint(), new Point(-3, odometry.getPoint().getY() + 0.01)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    strafeTurn(0, angleOffset, new Point(-3.3, 3));
                    strafeTurn(0, angleOffset, new Point(-2.35, 3));
                    path.clear();
                    strafeTurn(-odometry.getAngle(), angleOffset, new Point(-4, 4));
                    strafeTurn(-odometry.getAngle(), angleOffset, new Point(-4.6, 4));
                    path.add(new Line(odometry.getPoint(), new Point(-4.5, 0)));
                }
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(-4.5, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                }
            }
        }
    }

    /**
     * Forward-facing Pure Pursuit follower
     * @param path Path to follow
     * @throws InterruptedException Stop running if OpMode is stopped
     */
    public void follow(ArrayList<Line> path) throws InterruptedException {
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean done = false;
        for(int k = 0; k < path.size(); k++) {
            Line line = path.get(k);
            if(isStopRequested()) {
                throw new InterruptedException("OpMode Stopped");
            }
            // odometry.update();
            ArrayList<Point> nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
            boolean thing = false;
            while(opModeIsActive() && nextPoint.size() > 0) {
                while(opModeIsActive() && !Functions.isPassed(line, odometry.getPoint(), nextPoint.get(0))) {
                    double relAngle = odometry.getPoint().angle(nextPoint.get(0), AngleUnit.DEGREES) - odometry.getAngle();
                    if (Math.abs(relAngle + 360) < Math.abs(relAngle)) {
                        relAngle += 360;
                    }
                    if (Math.abs(relAngle - 360) < Math.abs(relAngle)) {
                        relAngle -= 360;
                    }
                    double drive = -Math.cos(Math.toRadians(relAngle));
                    double turn = turnCoeff * relAngle;
                    double angle = 0;
                    if(thing) {
                        Point myPos = odometry.getPoint();
                        Line perpLine = new Line(odometry.getPoint(), new Point(odometry.getPoint().getX() + 1, (-1.0 / line.getSlope()) + odometry.getPoint().getY()));
                        double x = (perpLine.getYInt() - line.getYInt()) / (line.getSlope() - perpLine.getSlope());
                        Point target = new Point(x, (perpLine.getSlope() * x) + perpLine.getYInt());
                        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
                        if (displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
                            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                            if (PIDd != -displacement) {
                                angle = 0.5 * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                                drive += 0.5 * PIDd;
                            }
                        }
                    }
                    double scaleFactor;
                    if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
                        scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
                    } else {
                        scaleFactor = Globals.MAX_SPEED;
                    }
                    /*
                    double scaleFactor;
                    if (Math.max(Math.abs((drive + turn)), Math.abs((drive - turn))) > 1) {
                        scaleFactor = Globals.MAX_SPEED / (Math.max(Math.abs(drive + turn), Math.abs(drive - turn)));
                    } else {
                        scaleFactor = Globals.MAX_SPEED;
                    }
                    */
                    robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
                    odometry.update();
                }
                stopped = odometry.getPoint();
                stopTarget = nextPoint.get(0);
                line = new Line(odometry.getPoint(), line.getPoint2());
                nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
                thing = true;
                if(k == path.size() - 1 && nextPoint.size() == 0 && !done) {
                    nextPoint.add(line.getPoint2());
                    done = true;
                }
            }
        }
        robot.setDrivePower(0, 0, 0, 0);
    }

    /**
     * Backward-facing Pure Pursuit follower
     * @param path Path to follow
     * @throws InterruptedException Stop running if OpMode is stopped
     */
    public void backFollow(ArrayList<Line> path) throws InterruptedException {
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for(int k = 0; k < path.size(); k++) {
            Line line = path.get(k);
            if(isStopRequested()) {
                throw new InterruptedException("OpMode Stopped");
            }
            // odometry.update();
            ArrayList<Point> nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
            boolean thing = false;
            while (opModeIsActive() && nextPoint.size() > 0) {
                while (opModeIsActive() && !Functions.isPassed(line, odometry.getPoint(), nextPoint.get(0))) {
                    double relAngle = odometry.getPoint().angle(nextPoint.get(0), AngleUnit.DEGREES) - odometry.getAngle();
                    if (Math.abs(relAngle + 360) < Math.abs(relAngle)) {
                        relAngle += 360;
                    }
                    if (Math.abs(relAngle - 360) < Math.abs(relAngle)) {
                        relAngle -= 360;
                    }
                    double drive = -Math.cos(Math.toRadians(relAngle));
                    double turn = turnCoeff * (relAngle - (180 * Math.signum(relAngle)));
                    double angle = 0;
                    if(thing) {
                        Point myPos = odometry.getPoint();
                        Line perpLine = new Line(odometry.getPoint(), new Point(odometry.getPoint().getX() + 1, (-1.0 / line.getSlope()) + odometry.getPoint().getY()));
                        double x = (perpLine.getYInt() - line.getYInt()) / (line.getSlope() - perpLine.getSlope());
                        Point target = new Point(x, (perpLine.getSlope() * x) + perpLine.getYInt());
                        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
                        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
                            double PIDd = -Math.cos((myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) - (Math.toRadians(180) * Math.signum(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())))) * displacement;
                            if (PIDd != -displacement) {
                                angle = 0.5 * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                                drive += 0.5 * PIDd;
                            }
                        }
                    }
                    double scaleFactor;
                    if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
                        scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
                    } else {
                        scaleFactor = Globals.MAX_SPEED;
                    }
                    robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
                    odometry.update();
                }
                line = new Line(odometry.getPoint(), line.getPoint2());
                nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
                thing = true;
                if(k == path.size() - 1 && nextPoint.size() == 0) {
                    nextPoint.add(line.getPoint2());
                }
            }
        }
        robot.setDrivePower(0, 0, 0, 0);
    }

    /**
     * Sideways-facing (90º) Pure Pursuit follower
     * @param path Path to follow
     * @throws InterruptedException Stop running if OpMode is stopped
     */
    public void sideFollow(ArrayList<Line> path) throws InterruptedException {
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for(int k = 0; k < path.size(); k++) {
            Line line = path.get(k);
            if(isStopRequested()) {
                throw new InterruptedException("OpMode Stopped");
            }
            // odometry.update();
            ArrayList<Point> nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
            boolean thing = false;
            while (opModeIsActive() && nextPoint.size() > 0) {
                while (opModeIsActive() && !Functions.isPassed(line, odometry.getPoint(), nextPoint.get(0))) {
                    double relAngle = odometry.getPoint().angle(nextPoint.get(0), AngleUnit.DEGREES) - odometry.getAngle();
                    if (Math.abs(relAngle + 360) < Math.abs(relAngle)) {
                        relAngle += 360;
                    }
                    if (Math.abs(relAngle - 360) < Math.abs(relAngle)) {
                        relAngle -= 360;
                    }
                    double drive = -Math.cos(Math.toRadians(relAngle));
                    double turn = turnCoeff * (relAngle - (90 * Math.signum(relAngle)));
                    double angle = 0;
                    if(thing) {
                        Point myPos = odometry.getPoint();
                        Line perpLine = new Line(odometry.getPoint(), new Point(odometry.getPoint().getX() + 1, (-1.0 / line.getSlope()) + odometry.getPoint().getY()));
                        double x = (perpLine.getYInt() - line.getYInt()) / (line.getSlope() - perpLine.getSlope());
                        Point target = new Point(x, (perpLine.getSlope() * x) + perpLine.getYInt());
                        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
                        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
                            double PIDd = -Math.cos((myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) - (Math.toRadians(90) * Math.signum(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())))) * displacement;
                            if (PIDd != -displacement) {
                                angle = 0.5 * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                                drive += 0.5 * PIDd;
                            }
                        }
                    }
                    double scaleFactor;
                    if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
                        scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
                    } else {
                        scaleFactor = Globals.MAX_SPEED;
                    }
                    robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
                    odometry.update();
                }
                line = new Line(odometry.getPoint(), line.getPoint2());
                nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
                thing = true;
                if(k == path.size() - 1 && nextPoint.size() == 0) {
                    nextPoint.add(line.getPoint2());
                }
            }
        }
        robot.setDrivePower(0, 0, 0, 0);
    }

    /**
     * Turn to face a certain point on the field
     * @param newPoint Point to now face
     */
    private void turnToPoint(Point newPoint) {
        double angle;
        Point currPoint = odometry.getPoint();
        try {
            angle = currPoint.angle(newPoint, AngleUnit.DEGREES);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = angle - robot.getAngle();
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance);
        }
    }

    /**
     * Turn so that the back of the robot faces a certain point on the field
     * @param newPoint Point to now face backwards
     */
    private void turnBackToPoint(Point newPoint) {
        double angle;
        Point currPoint = odometry.getPoint();
        try {
            angle = currPoint.angle(newPoint, AngleUnit.DEGREES);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = angle - robot.getAngle();
        turnDistance -= 180 * Math.signum(turnDistance);
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance);
        }
    }

    /**
     * Turn a certain, specified amount
     * @param angle Amount to turn
     */
    private void turn(double angle) {
        if(Math.abs(angle) < 0.1 || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double oldAngle;
        double angleIntended;
        double robotAngle;
        double lastError;
        double error = 0;
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = robot.getAngle();
        oldAngle = robotAngle;
        angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle)) {
                odometry.update();
                if(oldAngle > 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    lastError = error;
                    error = Math.abs(robotAngle - angleIntended);
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(Math.min(-0.02 * error, -0.1), Math.min(-0.02 * error, -0.1), Math.max(0.02 * error, 0.1), Math.max(0.02 * error, 0.1));
                robotAngle = robot.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        else if(opModeIsActive() && angleIntended > robotAngle) { // Right turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle)) {
                odometry.update();
                if(oldAngle < 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    error = Math.abs(robotAngle - angleIntended);
                    if(error > 180) {
                        lastError = error;
                        error = Math.abs(robotAngle + angleIntended);
                        if(lastError != 0 && error > lastError) {
                            error = lastError;
                        }
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(Math.max(0.02 * error, 0.1), Math.max(0.02 * error, 0.1), Math.min(-0.02 * error, -0.1), Math.min(-0.02 * error, -0.1));
                robotAngle = robot.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        odometry.update();
        resetEncoders();
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
    }

    /**
     * Reset drive encoders and sets them to RUN_USING_ENCODERS mode
     */
    private void resetEncoders() { // Reset encoder values and set encoders to "run using encoder" mode
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Get the direction in which a certain motor is traveling
     * @param motor Motor on which to perform check
     * @return The direction of the motor (-1 = backwards; 1 = forward; 0 = stationary)
     */
    private double getPower(DcMotor motor) {
        try {
            double power = (Math.abs(motor.getPower()) / motor.getPower()) * (Math.abs(motor.getTargetPosition()) - Math.abs(motor.getCurrentPosition())) / 500.0;
            if(Math.abs(power) >= 0.1) {
                return(Range.clip(power, -1, 1));
            }
            else if(power != 0){
                return(0.1 * (power < 0 ? -1 : 1));
            }
            else {
                return 0;
            }
        }
        catch(Exception p_exception) {
            return 0;
        }
    }

    /**
     * Move to a certain point linearly using encoders to assess accuracy
     * @param newPoint Point to which to travel
     */
    private void runToPoint(Point newPoint) {
        if(!opModeIsActive()) {
            return;
        }
        Point currPoint = odometry.getPoint();
        double angle;
        try {
            angle = currPoint.angle(newPoint, AngleUnit.DEGREES);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = angle - robot.getAngle();
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance);
        }
        int distance = currPoint.distance(newPoint);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance / 1500.0));
        // while(!gamepad1.a && opModeIsActive()) {}
    }

    /**
     * Move a certain distance using drive encoders
     * @param pos Distance to move (in encoder ticks)
     * @param speed Maximum speed for movement
     * @param timeLimit Maximum time to spend on movement
     */
    private void encoderMovePreciseTimed(int pos, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        RevBulkData data = robot.bulkRead();
        if(pos == 0 && timeLimit >= 0.1) {
            pos = -((data.getMotorCurrentPosition(robot.rf) + data.getMotorCurrentPosition(robot.lf) + data.getMotorCurrentPosition(robot.lb) + data.getMotorCurrentPosition(robot.rb)) / 4);
        }
        resetEncoders();
        if(opModeIsActive()) {
            if (opModeIsActive() && robot.rb != null && robot.rf != null && robot.lb != null && robot.lf != null) {
                robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rb.setTargetPosition(pos);
                robot.rf.setTargetPosition(pos);
                robot.lf.setTargetPosition(pos);
                robot.lb.setTargetPosition(pos);
                double startAngle = robot.getAngle();
                ElapsedTime delta = new ElapsedTime();
                double Kp = 0.0215;
                double Ki = 0.005;
                double Kd = 0.01;
                double i = 0;
                double lastError = 0;
                double maxSpeed = Globals.MAX_SPEED;
                ElapsedTime limitTest = new ElapsedTime();
                try {
                    robot.setDrivePower(Math.signum(pos) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed);
                } catch (Exception p_exception) {
                    robot.setDrivePower(speed, speed, speed, speed);
                }
                while((Math.abs(((data.getMotorCurrentPosition(robot.rf) + data.getMotorCurrentPosition(robot.lf) + data.getMotorCurrentPosition(robot.lb) + data.getMotorCurrentPosition(robot.rb)) / 4)) <= Math.abs(pos)) && opModeIsActive()) {
                    odometry.update();
                    data = robot.bulkRead();
                    double error = Math.abs(data.getMotorCurrentPosition(robot.rb) - robot.rb.getTargetPosition()) / 500.0;
                    if(error <= 0.25) {
                        error = 0.25;
                    }
                    if(!(speed >= maxSpeed) && !(error <= speed)) {
                        speed += 0.05;
                    }
                    else {
                        speed = Math.min(error, Math.abs(maxSpeed));
                    }
                    if(Math.abs(robot.getAngle() - startAngle) >= 7) {
                        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        double angleError = robot.getAngle() - startAngle;
                        try {
                            if(Math.abs(angleError) > 180 && (Math.abs(robot.getAngle()) / robot.getAngle()) != (Math.abs(startAngle) / startAngle)) {
                                angleError += angleError > 0 ? -360 : 360;
                            }
                        }
                        catch (Exception p_exception) {} // If an error happens, that means that either our current angle or initial angle was zero, so the error calculation should be accurate anyway
                        double right = getPower(robot.rb);
                        double left = getPower(robot.lf);
                        try {
                            robot.setDrivePower((Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed);
                        }
                        catch(Exception p_exception) {
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        double deltaError = angleError - lastError;
                        i += delta.time() * deltaError;
                        double changePower = (Kp * angleError) + (Ki * i) + (Kd * deltaError / delta.time());
                        right += changePower;
                        left -= changePower;
                        double max = Math.max(Math.abs(right), Math.max(Math.abs(left), speed));
                        right /= Math.abs(max);
                        left /= Math.abs(max);
                        robot.setDrivePower(left, left, right, right);
                        lastError = angleError;
                        delta.reset();
                    }
                    else {
                        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        try {
                            robot.setDrivePower((Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed);
                        } catch (Exception p_exception) {
                            robot.setDrivePower(speed, speed, speed, speed);
                        }
                    }
                }
                if (limitTest.time() > timeLimit) {
                    robot.rb.setTargetPosition((robot.rb.getCurrentPosition()));
                    robot.rf.setTargetPosition((robot.rf.getCurrentPosition()));
                    robot.lb.setTargetPosition((robot.lb.getCurrentPosition()));
                    robot.lf.setTargetPosition((robot.lf.getCurrentPosition()));
                }
                robot.setDrivePower(0, 0, 0, 0);
            }
        }
        else {
            throw new IllegalStateException();
        }
    }

    /**
     * Simultaneously strafe to a certain point and turn to a certain angle
     * @param angle Amount to turn during movement
     * @param angleOffset Turning tolerance
     * @param target Point to which to strafe
     */
    private void strafeTurn(double angle, double angleOffset, Point target) {
        if(Math.abs(angle) < 0.1 || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            double displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            if(Math.abs(displacement) < 0.15) {
                return;
            }
            else {
                wheels.update(robot, target, odometry, angle, AngleUnit.DEGREES);
            }
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double oldAngle;
        double angleIntended;
        double robotAngle;
        double lastError;
        double error = 0;
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = robot.getAngle();
        oldAngle = robotAngle;
        angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle)) {
                odometry.update();
                double[] strafePows = wheels.calcUpdate(target, odometry);
                if(oldAngle > 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    lastError = error;
                    error = Math.abs(robotAngle - angleIntended);
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                double scaleFactor;
                if(Math.max(Math.abs(strafePows[0] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs(strafePows[1] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs((strafePows[2] + Math.max(0.02 * error, 0.1))), Math.abs((strafePows[3] + Math.max(0.02 * error, 0.1)))))) > 1) {
                    scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(strafePows[0] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs(strafePows[1] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs((strafePows[2] + Math.max(0.02 * error, 0.1))), Math.abs((strafePows[3] + Math.max(0.02 * error, 0.1))))));
                } else {
                    scaleFactor = Globals.MAX_SPEED;
                }
                robot.setDrivePower(scaleFactor * (strafePows[0] + Math.min(-0.02 * error, -0.1)), scaleFactor * (strafePows[1] + Math.min(-0.02 * error, -0.1)), scaleFactor * (strafePows[2] + Math.max(0.02 * error, 0.1)), scaleFactor * (strafePows[3] + Math.max(0.02 * error, 0.1)));
                robotAngle = robot.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        else if(opModeIsActive() && angleIntended > robotAngle) { // Right turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle)) {
                odometry.update();
                double[] strafePows = wheels.calcUpdate(target, odometry);
                if(oldAngle < 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    error = Math.abs(robotAngle - angleIntended);
                    if(error > 180) {
                        lastError = error;
                        error = Math.abs(robotAngle + angleIntended);
                        if(lastError != 0 && error > lastError) {
                            error = lastError;
                        }
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                double scaleFactor;
                if(Math.max(Math.abs(strafePows[0] + Math.max(0.02 * error, 0.1)), Math.max(Math.abs(strafePows[1] + Math.max(0.02 * error, 0.1)), Math.max(Math.abs((strafePows[2] + Math.min(-0.02 * error, -0.1))), Math.abs((strafePows[3] + Math.min(-0.02 * error, -0.1)))))) > 1) {
                    scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(strafePows[0] + Math.max(0.02 * error, 0.1)), Math.max(Math.abs(strafePows[1] + Math.max(0.02 * error, 0.1)), Math.max(Math.abs((strafePows[2] + Math.min(-0.02 * error, -0.1))), Math.abs((strafePows[3] + Math.min(-0.02 * error, -0.1))))));
                } else {
                    scaleFactor = Globals.MAX_SPEED;
                }
                robot.setDrivePower(scaleFactor * (strafePows[0] + Math.max(0.02 * error, 0.1)), scaleFactor * (strafePows[1] + Math.max(0.02 * error, 0.1)), scaleFactor * (strafePows[2] + Math.min(-0.02 * error, -0.1)), scaleFactor * (strafePows[3] + Math.min(-0.02 * error, -0.1)));
                robotAngle = robot.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        odometry.update();
        double displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
        while(Math.abs(displacement) > 0.15) {
            wheels.update(robot, target, odometry, angle, AngleUnit.DEGREES);
            displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
        }
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
    }
}
