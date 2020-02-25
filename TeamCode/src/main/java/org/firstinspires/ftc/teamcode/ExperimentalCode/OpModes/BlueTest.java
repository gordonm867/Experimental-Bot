package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.GOFException;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.DetectionPipeline;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.RobotState;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Autonomous(name="BLUE Drive-Auto", group="Trash")
@Config
//@Disabled
public class BlueTest extends MyOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);

    public static int skystone = 2;

    private boolean founddone = false;

    Point target;
    Point lastPoint;

    public static int index = 0;

    private int iterations = 0;

    double lasttime = System.currentTimeMillis();
    double time = System.currentTimeMillis();
    double lastdisplacement = 0;

    ArrayList<RobotState> targets = new ArrayList<>();

    private double opTime = System.currentTimeMillis();
    private double boxPos = 0;

    private OpenCvCamera phoneCam;


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
        if(boxPos != Globals.boxNeutral) {
            boxPos = Globals.boxNeutral;
            robot.liftbox(boxPos);
        }
        robot.unlockFoundation();
        robot.closeClamp();
        //Globals.boxDown = 0.395;

        odometry = Odometry.getInstance(robot);

        robot.enabled = true;

        robot.dropOdometry();
        odometry.reset();
        if(robot.ex != null) {
            robot.ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.ex.setTargetPosition(robot.lift.getCurrentPosition());
            robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(robot.lift != null) {
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setTargetPosition(robot.lift.getCurrentPosition());
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
        opTime = System.currentTimeMillis();
        wheels.setState(Drivetrain.State.ON);
        if(skystone == 0) {
            /* First Block */
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, false, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(-3.2, -3.4 + (2f/3f), 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 3000, true)); // 1
            targets.add(new RobotState(-3.5, -3.4 + (2f/3f), 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(-3.5, 4, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(-2.65, 4, 180, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 6

            /* Reposition */
            targets.add(new RobotState(-5.45, 4, 180, 0, 0.6, false, Globals.boxNeutral, 501, true)); // 7
            targets.add(new RobotState(-5.45, 1.75, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 8

            /* Second Block */
            //targets.add(new RobotState(-3.64 -1.25 + (2f / 3f), 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(-3.6, 1.75, -90, 0, 0, false, Globals.boxNeutral , 0, false)); // 9
            targets.add(new RobotState(-3.6, -1, -90, 0, 0, false, Globals.boxNeutral , 0, false)); // 9
            targets.add(new RobotState(-3.6, -2.9, -135, Globals.EXTEND_POS, 0, false, Globals.boxNeutral, 0, false)); // 9
            targets.add(new RobotState(-3.6, -3.2, -135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 750, true)); // 10

            /* Place */
            targets.add(new RobotState(-3.4, -1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(-3.4, 1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(-3.4, 1.85, -90, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 3000, true)); // 13

            targets.add(new RobotState(-3.4, -2, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 14
            targets.add(new RobotState(-2.8, -3, 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 3000, false)); // 15
            targets.add(new RobotState(-2.5, -3, 135, 0, 0, false, Globals.boxNeutral, 0, true)); // 16
            targets.add(new RobotState(-3.4, 1.85, -100, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 17
            targets.add(new RobotState(-3.4, 1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 18
        }
        else if(skystone == 1) {
            /* First Block */
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(-3.5, -3.1, 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 3000, true)); // 1
            targets.add(new RobotState(-3.5, -3.1, 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(-3.5, 4, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(-2.65, 4, 180, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 6

            /* Reposition */
            targets.add(new RobotState(-5.55, 4, 180, 0, 0.6, false, Globals.boxNeutral, 501, true)); // 7
            targets.add(new RobotState(-5.55, 1.75, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 8

            /* Second Block */
            //targets.add(new RobotState(-3.64 -1.25 + (2f / 3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(-3.5, 1.75, 90, 0, 0, false, Globals.boxNeutral , 0, false)); // 9
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, false, Globals.boxNeutral , 0, false)); // 10
            targets.add(new RobotState(-3.7, -1.5, 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, false)); // 11
            targets.add(new RobotState(-3.4, -1.5, 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 1500, true)); // 12

            /* Place */
            targets.add(new RobotState(-3.4, -1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 13
            targets.add(new RobotState(-3.4, 1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 14
            targets.add(new RobotState(-3.2, 1.85, -90, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 15

            targets.add(new RobotState(-3.3, -2, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 16
            targets.add(new RobotState(-2.8, -3, 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 3000, true)); // 17
            targets.add(new RobotState(-2.5, -3, 135, 0, 0, false, Globals.boxNeutral, 0, true)); // 18
            targets.add(new RobotState(-3.4, 1.85, -100, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 19
            targets.add(new RobotState(-3.4, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 20
        }
        else {
            /* First Block */
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(-3.2, -3.4 + (4f/3f), 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 3000, true)); // 1
            targets.add(new RobotState(-3.5, -3.4 + (4f/3f), 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(-3.5, 4, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(-2.65, 4, 180, Globals.EXTEND_POS, 0.5, false, Globals.boxNeutral, 750, true)); // 6

            /* Reposition */
            targets.add(new RobotState(-5.45, 4, 180, 0, 0.5, false, Globals.boxNeutral, 501, true)); // 7
            targets.add(new RobotState(-5.45, 1.75, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 8

            /* Second Block */
            //targets.add(new RobotState(-3.64 -1.25 + (2f / 3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(-3.3, 1.75, 90, 0, 0, false, Globals.boxNeutral , 0, true)); // 9
            targets.add(new RobotState(-3.3, -3 - (2f/3f), 90, 0, 0, false, Globals.boxNeutral , 0, true)); // 9
            targets.add(new RobotState(-2, -2.8 - (2f/3f), 90, Globals.EXTEND_POS, -1, false, Globals.boxDown, 750, true)); // 10

            /* Place */
            targets.add(new RobotState(-3.4, -1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(-3.4, 1, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(-3.4, 1.85, -90, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 13

            targets.add(new RobotState(-3.3, -1.5, -90, 0, 0, false, Globals.boxNeutral, 0, false)); // 14
            targets.add(new RobotState(-3, -2.2, 135, Globals.EXTEND_POS, -1, false, Globals.boxDown, 3000, true)); // 15
            targets.add(new RobotState(-2.8, -2.2, 135, 0, 0, false, Globals.boxNeutral, 0, true)); // 16
            targets.add(new RobotState(-3.4, 1.85, -100, Globals.EXTEND_POS, 0.6, false, Globals.boxNeutral, 750, true)); // 17
            targets.add(new RobotState(-3.4, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 18
        }
        boxPos = robot.boxlift.getPosition();
        robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loopOp() {
        double cycletest = System.currentTimeMillis();
        if(!founddone && index >= 8) {
            robot.unlockFoundation();
            founddone = true;
        }
        RobotState target = targets.get(index);
        if(skystone == 1 && index == 10 && odometry.getY() < -0.75) {
            target.setExtendPos(Globals.EXTEND_POS);
        }
        if((skystone != 2 && index == 2) || (skystone == 0 && index == 16)) {
            target.setY(odometry.getY());
        }
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
        double myAngle = odometry.getAngle();
        double angular = Functions.normalize(Math.abs(myAngle - target.getAngle()));
        RevBulkData data = robot.bulkRead();
        int expos = 0;
        if(Math.abs(opTime - System.currentTimeMillis()) >= 28000) {
            if(boxPos != Globals.boxNeutral) {
                boxPos = Globals.boxNeutral;
                robot.liftbox(boxPos);
            }
            robot.ex.setTargetPosition(0);
            robot.ex.setPower(0.5);
            target.setX(-3.3);
            target.setY(0);
            target.setTheta(90 * Math.signum(myAngle));
            while(opModeIsActive() && Math.abs(odometry.getY()) > 0.5) {
                data = robot.bulkRead();
                myAngle = odometry.getAngle();
                Globals.MAX_SPEED = 1.0;
                wheels.update(robot, target, odometry, target.getAngle(), myAngle, data);
            }
            while(opModeIsActive()) {
                double nTime = System.currentTimeMillis();
                if(Math.abs(odometry.getX()) < 3.8) {
                    robot.moveClamp(Clamp.MAX);
                }
                else {
                    robot.moveClamp(Clamp.MIN);
                }
                robot.setDrivePower(0, 0, 0, 0);
                telemetry.addData("Time elapsed", (nTime - opTime) / 1000f);
                telemetry.update();
            }
        }
        if(((target.isRequired() && target.isExact() && (displacement >= 0.25 || angular >= 3 || (robot.ex != null && Math.abs(expos - target.getExtendPos()) >= 300))) || (!target.isRequired() && !target.isExact() && (displacement >= 0.5)) || (target.isRequired() && !target.isExact() && (displacement >= 0.5 || (robot.ex != null && Math.abs(expos - target.getExtendPos()) >= 300))) || (!target.isRequired() && target.isExact() && (displacement >= 0.25 || angular >= 3)))) {
            if(targets.get(index).getIntakePower() > 0.1) {
                robot.moveClamp(Clamp.MIN);
            }
            if(targets.get(index).getBoxPos() == Globals.boxDown || index < targets.size() - 1 && targets.get(index + 1).getBoxPos() == Globals.boxDown) {
                robot.moveClamp(Clamp.MAX);
            }
            if(index == 8) {
                robot.moveClamp(Clamp.MIN);
            }
            Point startPoint = odometry.getPoint();
            double itertime = System.currentTimeMillis();
            wheels.update(robot, target, odometry, target.getAngle(), myAngle, data);
            if(displacement > 0.5 && iterations > 20 && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= (0.2 * Globals.MAX_SPEED)) { // Move on....
                if(odometry.getY() > -2.4 && odometry.getY() < 2.4) {
                    if(Math.abs(odometry.getX()) < 3.8) {
                        while (Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= (0.2 * Globals.MAX_SPEED)) {
                            myAngle = odometry.getAngle();
                            startPoint = odometry.getPoint();
                            iterations++;
                            lastdisplacement = displacement;
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
                            lastdisplacement = displacement;
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
            if(robot.ex != null) {
                robot.ex.setPower(0.5);
                robot.ex.setTargetPosition(target.getExtendPos());
            }
            if(index == 6) {
                Globals.MAX_SPEED = 0.6;
                Globals.MIN_SPEED = 0.4;
            }
            else if(index == 7) {
                Globals.MAX_SPEED = 0.6;
            }
            else if(index == 1 && Math.abs(odometry.getX()) < 3.5) {
                Globals.MAX_SPEED = 0.25;
            }
            else {
                Globals.MAX_SPEED = 1.0;
                Globals.MIN_SPEED = 0.25;
            }
            if(target.getIntakePower() < 0) {
                robot.setInPower(target.getIntakePower());
            }
            if(robot.hasBlock() && target.getIntakePower() < -0.5) {
                index++;
                iterations = 0;
                time = System.currentTimeMillis();
                target = targets.get(index);
            }
            if(boxPos != target.getBoxPos()) {
                boxPos = target.getBoxPos();
                robot.liftbox(boxPos);
            }
            double cycle = 1000f / (System.currentTimeMillis() - cycletest);
            telemetry.addData("Cycle time", Math.round(cycle) + " Hz");
            telemetry.update();
            iterations++;
            lastdisplacement = displacement;
            lastPoint = startPoint;
            lasttime = itertime;
        }
        else {
            if(boxPos != target.getBoxPos()) {
                boxPos = target.getBoxPos();
                robot.liftbox(boxPos);
            }
            robot.setInPower(0);
            if(target.getDelay() != 0) {
                robot.setDrivePower(0, 0, 0, 0);
                double nowTime = System.currentTimeMillis();
                while(opModeIsActive() && Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                    myAngle = odometry.getAngle();
                    data = robot.bulkRead();
                    if(Math.abs(nowTime - System.currentTimeMillis()) >= 200 || index < 10) {
                        robot.setInPower(target.getIntakePower());
                    }
                    if(Math.abs(opTime - System.currentTimeMillis()) >= 28000) {
                        if(boxPos != Globals.boxNeutral) {
                            boxPos = Globals.boxNeutral;
                            robot.liftbox(boxPos);
                        }
                        robot.ex.setTargetPosition(0);
                        robot.ex.setPower(0.5);
                        target.setX(-3.3);
                        target.setY(0);
                        target.setTheta(90 * Math.signum(myAngle));
                        while(opModeIsActive() && Math.abs(odometry.getY()) > 0.5) {
                            data = robot.bulkRead();
                            myAngle = odometry.getAngle();
                            Globals.MAX_SPEED = 1.0;
                            wheels.update(robot, target, odometry, target.getAngle(), myAngle, data);
                        }
                        while(opModeIsActive()) {
                            double nTime = System.currentTimeMillis();
                            if(Math.abs(odometry.getX()) < 3.8) {
                                robot.moveClamp(Clamp.MAX);
                            }
                            else {
                                robot.moveClamp(Clamp.MIN);
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            telemetry.addData("Time elapsed", (nTime - opTime) / 1000f);
                            telemetry.update();
                        }
                    }
                    if(target.getIntakePower() < -0.2) {
                        if(robot.hasBlock() || Math.abs(odometry.getX()) < 1.5 || Math.abs(System.currentTimeMillis() - nowTime) >= 2000) {
                            robot.setDrivePower(0, 0, 0, 0);
                            robot.setInPower(-0.75);
                            target.setDelay(0);
                            if(robot.hasBlock()) {
                                sleep(1000);
                            }
                            Globals.MAX_SPEED = 1.0;
                            break;
                        }
                        else {
                            if(Math.abs(odometry.getX()) < 3.8) {
                                robot.moveClamp(Clamp.MAX);
                            }
                            else {
                                robot.moveClamp(Clamp.MIN);
                            }
                            Globals.MAX_SPEED = 0.3;
                            wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(myAngle))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(myAngle)))), odometry, target.getAngle(), myAngle, data);
                        }
                    }
                    if(index == 6) {
                        robot.lockFoundation();
                        robot.moveClamp(Clamp.MIN);
                    }
                    else if(index == 7) {
                        robot.unlockFoundation();
                    }
                    if(target.getIntakePower() > 0.3) {
                        if(index < 10) {
                            robot.lockFoundation();
                            robot.moveClamp(Clamp.MIN);
                            break;
                        }
                        if(robot.hasBlock() && target.getIntakePower() > 0.2) {
                            robot.setInPower(0.33);
                            if(index < 10) {
                                robot.lockFoundation();
                                robot.moveClamp(Clamp.MIN);
                                break;
                            }
                            else {
                                robot.moveClamp(Clamp.MIN);
                            }
                            while(opModeIsActive() && robot.hasBlock()) {
                                myAngle = odometry.getAngle();
                                if(Math.abs(opTime - System.currentTimeMillis()) >= 28000) {
                                    if(boxPos != Globals.boxNeutral) {
                                        boxPos = Globals.boxNeutral;
                                        robot.liftbox(boxPos);
                                    }
                                    robot.ex.setTargetPosition(0);
                                    robot.ex.setPower(0.5);
                                    target.setX(-3.3);
                                    target.setY(0);
                                    target.setTheta(90 * Math.signum(myAngle));
                                    while(opModeIsActive() && Math.abs(odometry.getY()) > 0.5) {
                                        myAngle = odometry.getAngle();
                                        data = robot.bulkRead();
                                        Globals.MAX_SPEED = 1.0;
                                        wheels.update(robot, target, odometry, target.getAngle(), myAngle, data);
                                    }
                                    while(opModeIsActive()) {
                                        double nTime = System.currentTimeMillis();
                                        if(Math.abs(odometry.getX()) < 3.8) {
                                            robot.moveClamp(Clamp.MAX);
                                        }
                                        else {
                                            robot.moveClamp(Clamp.MIN);
                                        }
                                        robot.setDrivePower(0, 0, 0, 0);
                                        telemetry.addData("Time elapsed", (nTime - opTime) / 1000f);
                                        telemetry.update();
                                    }
                                }
                                sleep(100);
                            }
                            sleep(500);
                            if(robot.hasBlock()) {
                                while(opModeIsActive() && robot.hasBlock()) {
                                    myAngle = odometry.getAngle();
                                    if(Math.abs(opTime - System.currentTimeMillis()) >= 28000) {
                                        if(boxPos != Globals.boxNeutral) {
                                            boxPos = Globals.boxNeutral;
                                            robot.liftbox(boxPos);
                                        }
                                        robot.ex.setTargetPosition(0);
                                        robot.ex.setPower(0.5);
                                        target.setX(-3.3);
                                        target.setY(0);
                                        target.setTheta(90 * Math.signum(myAngle));
                                        while(opModeIsActive() && Math.abs(odometry.getY()) > 0.5) {
                                            myAngle = odometry.getAngle();
                                            data = robot.bulkRead();
                                            Globals.MAX_SPEED = 1.0;
                                            wheels.update(robot, target, odometry, target.getAngle(), myAngle, data);
                                        }
                                        while(opModeIsActive()) {
                                            double nTime = System.currentTimeMillis();
                                            if(Math.abs(odometry.getX()) < 3.8) {
                                                robot.moveClamp(Clamp.MAX);
                                            }
                                            else {
                                                robot.moveClamp(Clamp.MIN);
                                            }
                                            robot.setDrivePower(0, 0, 0, 0);
                                            telemetry.addData("Time elapsed", (nTime - opTime) / 1000f);
                                            telemetry.update();
                                        }
                                    }
                                    sleep(500);
                                }
                            }
                            break;
                        }
                        else if(index < 10) {
                            break;
                        }
                    }
                    Globals.MAX_SPEED = 1.0;
                }
            }
            index++;
            if(index >= 8) {
                robot.unlockFoundation();
            }
            time = System.currentTimeMillis();
            iterations = 0;
            if(index != targets.size() && targets.get(index).getX() <= -4.5) {
                robot.moveClamp(Clamp.MIN);
            }
            if(index == targets.size()) {
                double nowTime = System.currentTimeMillis();
                while(opModeIsActive()) {
                    if(Math.abs(odometry.getX()) < 3.8) {
                        robot.moveClamp(Clamp.MAX);
                    }
                    else {
                        robot.moveClamp(Clamp.MIN);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    telemetry.addData("Time elapsed", nowTime - opTime);
                    telemetry.update();
                }
            }
        }
    }

    public void stopOp() {
        robot.setDrivePower(0, 0, 0, 0);
        robot.ex.setPower(0);
        robot.setInPower(0);
    }
}