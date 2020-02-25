package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.StoneClamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.ClampedStates;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.DetectionPipeline;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.RobotState;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Autonomous(name="BLUE Clamp-Auto", group="Trash")
@Config
//@Disabled
public class BlueClampAuto extends MyOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.ON);

    public static int skystone = 2;

    Point lastPoint;

    public static int index = 0;

    private int iterations = 0;

    double lasttime = System.currentTimeMillis();
    double time = System.currentTimeMillis();
    double lastdisplacement = Integer.MAX_VALUE;

    ArrayList<ClampedStates> targets = new ArrayList<>();

    private double opTime = System.currentTimeMillis();

    private OpenCvCamera phoneCam;

    private int foundationIndex = 12;

    public void initOp() {
        Globals.START_THETA = 180;
        Globals.START_X = -5.25;
        /* Initialize Hardware */
        index = 0;
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.moveClamp(Clamp.MIN);
        robot.unlockFoundation();
        //Globals.boxDown = 0.395;

        odometry = Odometry.getInstance(robot);
        lastPoint = odometry.getPoint();

        robot.enabled = true;

        robot.openClaw();
        robot.dropOdometry();
        odometry.reset();
        Globals.MAX_SPEED = 1.0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        phoneCam.openCameraDevice();
        DetectionPipeline pipeline = new DetectionPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        while(!pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }
        while(!isStarted() && !isStopRequested()) {
            robot.setDrivePower(0, 0, 0, 0);
            telemetry.addData("Status", "Initialized");
            skystone = pipeline.getBestCol();
            if(skystone == 0) {
                skystone = 2;
            }
            else if(skystone == 2) {
                skystone = 0;
            }
            telemetry.addData("Skystone", skystone == 0 ? "Left" : skystone == 1 ? "Center" : "Right");
            telemetry.update();
        }
    }

    public void startOp() {
        robot.openClaw();
        opTime = System.currentTimeMillis();
        wheels.setState(Drivetrain.State.ON);
        double[] stone = new double[12];
        stone[0] = -1.725; // -1.375
        stone[1] = -2.45;
        stone[2] = -3.1;
        stone[3] = -3.675;
        stone[4] = -4.35;
        stone[5] = -4.8;

        double lowplace = 3;

        if(skystone == 0) {
            targets.add(new ClampedStates(-5.25, -3.2, 180, false, StoneClamp.UP, false, 0, true));

            targets.add(new ClampedStates(-3.25, stone[3], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[3], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + (4f/3f) - 0.2, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[0], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[0], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + (2f/3f), 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[1], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[1], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + 0.5, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[2], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[2], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.65, lowplace + 0.5, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-2.4, lowplace + 1, 180, true, 0, false, 250, false));
            targets.add(new ClampedStates(-5.25, lowplace + 1.2, 180, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-5.25, 1.3, 180, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-3.6, 1.3, 180, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-3.75, 0, -90, false, StoneClamp.UP, false, 0, true));
        }
        else if(skystone == 1) {
            targets.add(new ClampedStates(-5.25, -3.2, 180, false, StoneClamp.UP, false, 0, true));

            targets.add(new ClampedStates(-3.25, stone[4], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[4], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + (4f/3f) - 0.2, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[1], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[1], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + (2f/3f), 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[0] + 0.05, 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[0], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + 0.5, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[2], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[2], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.65, lowplace + 0.5, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-2.4, lowplace + 1, 180, true, 0, false, 250, false));
            targets.add(new ClampedStates(-5.25, lowplace + 1.2, 180, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-5.25, 1.3, 180, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-3.6, 1.3, 180, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-3.75, 0, -90, false, StoneClamp.UP, false, 0, true));
        }
        else {
            targets.add(new ClampedStates(-5.25, -3.2, 180, false, StoneClamp.UP, false, 0, true));

            targets.add(new ClampedStates(-3.25, stone[5], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[5], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + (4f/3f) - 0.2, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[2], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[2], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace + (2f/3f), 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[0], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[0], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.85, lowplace, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-3.25, stone[1], 90, false, StoneClamp.GRAB, true, 1000, true));
            targets.add(new ClampedStates(-3.8, stone[1], 90, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-2.9, lowplace, 90, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-2.4, lowplace + 1.2, 180, true, StoneClamp.UP, false, 250, false));
            targets.add(new ClampedStates(-5.25, lowplace + 1.4, 180, false, StoneClamp.UP, false, 250, false));

            targets.add(new ClampedStates(-5.25, 1.3, 180, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-3.6, 1.3, 180, false, StoneClamp.UP, false, 0, false));
            targets.add(new ClampedStates(-3.75, 0, -90, false, StoneClamp.UP, false, 0, true));
        }
    }

    public void loopOp() {
        ClampedStates target = targets.get(index);
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
        if(index < foundationIndex && Math.abs(opTime - System.currentTimeMillis()) >= (24000 - (500 * (Math.abs(Math.sqrt(Math.pow(targets.get(foundationIndex).getX() - odometry.getX(), 2) + Math.pow(targets.get(foundationIndex).getY() - odometry.getY(), 2))))))) {
            if(odometry.getY() > 1) {
                index = foundationIndex;
                target = targets.get(index);
            }
        }
        else if(index == foundationIndex && Math.abs(opTime - System.currentTimeMillis()) >= (24000 - (500 * (Math.abs(Math.sqrt(Math.pow(targets.get(foundationIndex).getX() - odometry.getX(), 2) + Math.pow(targets.get(foundationIndex).getY() - odometry.getY(), 2)))))) && odometry.getY() > 1) {
            target.setTheta(Globals.START_THETA);
            target.setDelay(0);
        }
        if(odometry.getY() < -0.75 && target.getClawPos() != StoneClamp.UP) {
            robot.moveStoneBase(0.384);
            robot.openClaw();
        }
        else {
            robot.moveStoneBase(StoneClamp.UP);
        }
        if(Math.abs(odometry.getY()) < 2 && Math.abs(odometry.getY()) < Math.abs(target.getY()) - 0.5) {
            robot.closeClaw();
        }
        double myAngle = odometry.getAngle();
        double angular = Functions.normalize(Math.abs(myAngle - target.getAngle()));
        if(!((displacement < 0.05 && angular < 3) || (!target.isExact() && displacement < 0.5 && angular < 10))) {
            Point startPoint = odometry.getPoint();
            double itertime = System.currentTimeMillis();
            double delta = lastdisplacement - displacement;
            double velocity = Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0));
            RevBulkData data = robot.bulkRead();
            if(displacement < 0.15 || index >= foundationIndex + 1 || (Math.signum(target.getY()) == Math.signum(odometry.getY()))) {
                wheels.clupdate(robot, target, odometry, target.getAngle(), velocity, delta, myAngle, data);
            }
            else if(Math.abs(odometry.getX()) < 3.6 && Math.abs(odometry.getY()) > 3) {
                wheels.clupdate(robot, new Point(6 * Math.signum(odometry.getX()), target.getY()), odometry, target.getAngle(), velocity, delta, myAngle, data);
            }
            else if(Math.abs(odometry.getX()) > 3.9 && Math.abs(odometry.getY()) > 3) {
                wheels.clupdate(robot, new Point(3.5 * Math.signum(odometry.getX()), target.getY()), odometry, target.getAngle(), velocity, delta, myAngle, data);
            }
            else if(Math.abs(odometry.getX()) < 3.6) {
                while(Math.abs(odometry.getX()) < 3.6) {
                    robot.setDrivePower(-Math.signum(myAngle), Math.signum(myAngle), Math.signum(myAngle), -Math.signum(myAngle));
                    odometry.update(robot.bulkRead());
                }
                wheels.clupdate(robot, target, odometry, target.getAngle(), velocity, delta, myAngle, data);
            }
            else if(Math.abs(odometry.getX()) > 3.9) {
                while(Math.abs(odometry.getX()) > 3.9) {
                    robot.setDrivePower(Math.signum(myAngle), -Math.signum(myAngle), -Math.signum(myAngle), Math.signum(myAngle));
                    odometry.update(robot.bulkRead());
                }
                wheels.clupdate(robot, target, odometry, target.getAngle(), velocity, delta, myAngle, data);
            }
            else {
                wheels.clupdate(robot, new Point(odometry.getX(), target.getY()), odometry, target.getAngle(), velocity, delta, myAngle, data);
            }
            if(displacement > 0.5 && iterations > 20 && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= (0.2 * Globals.MAX_SPEED)) { // Move on....
                if(odometry.getY() > -0.9 && odometry.getY() < 0.9) {
                    if(Math.abs(odometry.getX()) < 3.6) {
                        while (Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= (0.2 * Globals.MAX_SPEED)) {
                            myAngle = odometry.getAngle();
                            startPoint = odometry.getPoint();
                            iterations++;
                            double[] pows = wheels.calcUpdate(target, odometry, target.getAngle(), myAngle);
                            robot.setDrivePower(-pows[0], -pows[1], -pows[2], -pows[3]);
                            double meTime = System.currentTimeMillis();
                            while (Math.abs(meTime - System.currentTimeMillis()) <= 100) {
                                odometry.update(robot.bulkRead());
                            }
                            lastPoint = startPoint;
                            lasttime = itertime;
                        }
                        robot.setDrivePower((-0.8 * Math.signum(myAngle) - 0.2), (0.8 * Math.signum(myAngle)) - 0.2, (0.8 * Math.signum(myAngle) - 0.2), (-0.8 * Math.signum(myAngle)) - 0.2);
                        double meTime = System.currentTimeMillis();
                        while (Math.abs(meTime - System.currentTimeMillis()) <= 250) {
                            odometry.update(robot.bulkRead());
                        }
                        iterations = 10;
                        time = System.currentTimeMillis();
                        if (Math.abs(target.getX()) >= 2.9) {
                            target.setX(target.getX());
                        }
                    }
                    else {
                        while(Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.1) {
                            myAngle = odometry.getAngle();
                            startPoint = odometry.getPoint();
                            iterations++;
                            double[] pows = wheels.calcUpdate(target, odometry, target.getAngle(), myAngle);
                            robot.setDrivePower(-pows[0], -pows[1], -pows[2], -pows[3]);
                            double meTime = System.currentTimeMillis();
                            while (Math.abs(meTime - System.currentTimeMillis()) <= 100) {
                                odometry.update(robot.bulkRead());
                            }
                            lastPoint = startPoint;
                            lasttime = itertime;
                        }
                        robot.setDrivePower((0.8 * Math.signum(myAngle) - 0.2), (-0.8 * Math.signum(myAngle)) - 0.2, (-0.8 * Math.signum(myAngle) - 0.2), (0.8 * Math.signum(myAngle)) - 0.2);
                        double meTime = System.currentTimeMillis();
                        while (Math.abs(meTime - System.currentTimeMillis()) <= 250) {
                            odometry.update(robot.bulkRead());
                        }
                        iterations = 10;
                        time = System.currentTimeMillis();
                        if(Math.abs(target.getX()) <= 4) {
                            target.setX(target.getX());
                        }
                    }
                }
            }
            iterations++;
            lastdisplacement = displacement;
            lastPoint = startPoint;
            lasttime = itertime;
        }
        else {
            if(target.isLocked()) {
                robot.lockFoundation();
            }
            else {
                robot.unlockFoundation();
            }
            if(target.getDelay() != 0) {
                robot.setDrivePower(0, 0, 0, 0);
                double nowTime = System.currentTimeMillis();
                while(opModeIsActive() && Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                    if(index == 0) {
                        robot.openClaw();
                    }
                    else if(Math.signum(odometry.getY()) == -1) {
                        robot.moveStoneBase(0.384);
                        double delay384 = System.currentTimeMillis();
                        while(opModeIsActive() && Math.abs(System.currentTimeMillis() - delay384) <= 100) {}
                        displacement = Math.abs(Math.sqrt(Math.pow(-2.9375 - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
                        double recTime = System.currentTimeMillis();
                        while(opModeIsActive() && displacement > 0.05 && Math.abs(recTime - System.currentTimeMillis()) <= 2500) {
                            Globals.MAX_SPEED = 0.35;
                            RevBulkData data = robot.bulkRead();
                            myAngle = odometry.getAngle();
                            Point startPoint = odometry.getPoint();
                            displacement = Math.abs(Math.sqrt(Math.pow(-2.9375 - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
                            double itertime = System.currentTimeMillis();
                            double delta = lastdisplacement - displacement;
                            double velocity = Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0));
                            wheels.clupdate(robot, new Point(odometry.getX() < -2.9375 ? -2.9375 : -1.9375, target.getY()), odometry, myAngle, velocity, delta, myAngle, data);
                            iterations++;
                            lastdisplacement = displacement;
                            lastPoint = startPoint;
                            lasttime = itertime;
                        }
                        Globals.MAX_SPEED = 1.0;
                        robot.setDrivePower(0, 0, 0, 0);
                        robot.moveStoneBase(StoneClamp.GRAB);
                        delay384 = System.currentTimeMillis();
                        while(opModeIsActive() && Math.abs(System.currentTimeMillis() - delay384) <= 500) {}
                        robot.closeClaw();
                        delay384 = System.currentTimeMillis();
                        while(opModeIsActive() && Math.abs(System.currentTimeMillis() - delay384) <= 300) {}
                    }
                    else {
                        robot.moveStoneBase(StoneClamp.FOUNDATION);
                        robot.openClaw();
                    }
                    if(index < foundationIndex && Math.abs(opTime - System.currentTimeMillis()) >= (24000 - (500 * (Math.abs(Math.sqrt(Math.pow(targets.get(foundationIndex).getX() - odometry.getX(), 2) + Math.pow(targets.get(foundationIndex).getY() - odometry.getY(), 2))))))) {
                        if(odometry.getY() > 1) {
                            index = foundationIndex;
                            break;
                        }
                    }
                    else if(index == foundationIndex && Math.abs(opTime - System.currentTimeMillis()) >= (24000 - (500 * (Math.abs(Math.sqrt(Math.pow(targets.get(foundationIndex).getX() - odometry.getX(), 2) + Math.pow(targets.get(foundationIndex).getY() - odometry.getY(), 2)))))) && odometry.getY() > 1) {
                        target.setTheta(Globals.START_THETA);
                    }
                    Globals.MAX_SPEED = 1.0;
                }
                robot.moveStoneBase(StoneClamp.UP);
            }
            index++;
            time = System.currentTimeMillis();
            iterations = 0;
            if(index == targets.size()) {
                double nowTime = System.currentTimeMillis();
                robot.moveClamp(Clamp.MAX);
                while(opModeIsActive()) {
                    robot.setDrivePower(0, 0, 0, 0);
                    telemetry.addData("Time elapsed", nowTime - opTime);
                    telemetry.update();
                }
            }
        }
    }

    public void stopOp() {
        robot.moveClamp(Clamp.MAX);
        robot.setDrivePower(0, 0, 0, 0);
        robot.ex.setPower(0);
        robot.setInPower(0);
    }
}