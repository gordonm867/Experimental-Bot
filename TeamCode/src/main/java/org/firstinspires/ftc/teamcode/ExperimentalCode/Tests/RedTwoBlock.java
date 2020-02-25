package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="RED Two Stone Autonomous", group="Trash")
@Disabled
public class RedTwoBlock extends MyOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);

    public static int skystone = 2;

    Point target;
    Point lastPoint;

    public static double TARGET_THETA = Globals.START_THETA;
    public static int READY = 0;
    public static int index = 0;
    public static int paused = 0;

    private int iterations = 0;

    double lasttime = System.currentTimeMillis();
    double time = System.currentTimeMillis();

    ArrayList<RobotState> targets = new ArrayList<>();

    private double opTime = System.currentTimeMillis();

    private OpenCvCamera phoneCam;


    public void initOp() {
        Globals.START_THETA = 0;
        Globals.START_X = 5.25;
        /* Initialize Hardware */
        index = 0;
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.moveClamp(Clamp.MIN);
        robot.liftbox(Globals.boxNeutral);
        robot.unlockFoundation();
        robot.closeClamp();
        Globals.boxDown = 0.395;

        odometry = Odometry.getInstance(robot);
        /*
        boxLift = new BoxLift(BoxLift.State.NEUTRAL);
        extension = new Extension(Extension.State.IDLE);
        mover = new FoundationMover(FoundationMover.State.UNLOCKED);
        intake = new Intake(Intake.State.IDLE);

        subsystems.add(boxLift);
        subsystems.add(extension);
        subsystems.add(mover);
        subsystems.add(intake);
         */

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
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(odometry.getPoint().equals(new Point(4, -2))) {
            Globals.MAX_SPEED = 0.4;
            READY = 1;
            target = targets.get(0);
            double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
            while(((displacement >= 0.15 || (Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()))) >= 1) && READY != 0)) {
                RevBulkData data = robot.bulkRead();
                odometry.update(data);
                displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
                TARGET_THETA = target.getAngle();
                if(Double.isNaN(target.getAngle())) {
                    TARGET_THETA = odometry.getAngle();
                }
                if((displacement >= 0.15 || (Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()))) >= 1) && READY != 0) {
                    wheels.update(robot, target, odometry, target.getAngle(), AngleUnit.DEGREES);
                }
            }
            index++;
            if(index == 16) {
                robot.unlockFoundation();
            }
            time = System.currentTimeMillis();
            READY = 0;
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
            telemetry.addData("Skystone", skystone == 0 ? "Right" : skystone == 1 ? "Center" : "Left");
            telemetry.update();
        }
    }

    public void startOp() {
        opTime = System.currentTimeMillis();
        wheels.setState(Drivetrain.State.ON);
        if(skystone == 0) {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(2.5, -3.6 + (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 2100, true)); // 1
            targets.add(new RobotState(3.4, -2.7 + (2f/3f), 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(3.6, 3.2, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(2.8, 3.2, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 6

            /* Second Block */
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 7
            targets.add(new RobotState(3.6, -1.25 + (2f/3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(3, -2 + (2f/3f), 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true)); // 9
            targets.add(new RobotState(2.7, -1.7 + (2f/3f), 150, Globals.EXTEND_POS, -1, true, Globals.boxDown, 2100, true)); // 10

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(3.6, 4, 180, Globals.EXTEND_POS, 0, false, Globals.boxNeutral, 0, false)); // 13
            targets.add(new RobotState(2.4, 4, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 500.01, false)); // 14

            /* Reposition */
            targets.add(new RobotState(5.5, 4.75, 180, 0, 1, true, Globals.boxNeutral, 501, true)); // 15
            targets.add(new RobotState(5.55, 2, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 16
            targets.add(new RobotState(3.5, 2, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 17

            /* Park */
            targets.add(new RobotState(3.8, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true)); // 18
        }
        else if(skystone == 1) {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(3.1, -3.4, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 2100, true)); // 1
            targets.add(new RobotState(3.4, -2.7, 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(3.6, 3.2, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(2.8, 3.2, 180, Globals.EXTEND_POS, 1, true, 0.43, 2000, true)); // 6

            /* Second Block */
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 7
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(3, -1.8, 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true)); // 9
            targets.add(new RobotState(2.7, -1.8, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 2100, true)); // 10

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(3.6, 4, 180, Globals.EXTEND_POS, 0, false, Globals.boxNeutral, 0, false)); // 13
            targets.add(new RobotState(2.4, 4, 180, Globals.EXTEND_POS, 1, false, 0.43, 500.01, false)); // 14

            /* Reposition */
            targets.add(new RobotState(5.5, 4.75, 180, 0, 1, true, Globals.boxNeutral, 501, true)); // 15
            targets.add(new RobotState(5.55, 2, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 16

            /* Park */
            targets.add(new RobotState(3.5, 2, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 17
            targets.add(new RobotState(3.8, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true)); // 18
        }
        else {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(2.7, -3.6 - (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 2100, true)); // 1
            targets.add(new RobotState(3.4, -2.7 - (2f/3f), 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(3.6, 3.2, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(3, 3.2, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 6

            /* Second Block */
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 7
            targets.add(new RobotState(3.6, -1.25 - (2f/3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(3, -2 - (2f/3f), 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true)); // 9
            targets.add(new RobotState(2.7, -2 - (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 2100, true)); // 10

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(3.6, 4, 180, Globals.EXTEND_POS, 0, false, Globals.boxNeutral, 0, false)); // 13
            targets.add(new RobotState(2.4, 4, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 500.01, false)); // 14

            /* Reposition */
            targets.add(new RobotState(5.5, 4.75, 180, 0, 1, true, Globals.boxNeutral, 501, true)); // 15
            targets.add(new RobotState(5.55, 2, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 16
            targets.add(new RobotState(3.5, 2, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 16

            /* Park */
            targets.add(new RobotState(3.8, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true)); // 17
        }
        for(RobotState target : targets) {
            target.setTheta(180 - target.getAngle());
        }
    }

    public void loopOp() {
        if(index >= 16) {
            robot.unlockFoundation();
        }
        RobotState target = targets.get(index);
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
        double angular = Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()));
        RevBulkData data = robot.bulkRead();
        int expos = data.getMotorCurrentPosition(robot.ex);
        if(paused == 0 && ((target.isRequired() && target.isExact() && (displacement >= 0.15 || angular >= 3 || (robot.ex != null && Math.abs(expos - target.getExtendPos()) >= 300))) || (!target.isRequired() && !target.isExact() && (displacement >= 0.5)) || (target.isRequired() && !target.isExact() && (displacement >= 0.5 || (robot.ex != null && Math.abs(expos - target.getExtendPos()) >= 300))) || (!target.isRequired() && target.isExact() && (displacement >= 0.15 || angular >= 3)))) {
            if(gamepad1.x) {
                paused = 1;
            }
            Point startPoint = odometry.getPoint();
            double itertime = System.currentTimeMillis();
            if(Math.abs(System.currentTimeMillis() - time) >= 2000) {
                if(target.getDelay() == 0 && target.isRequired() && ((target.isExact() && displacement < 0.15) || (!target.isExact() && displacement < 0.5))) {
                    index++;
                    if(index == 16) {
                        robot.unlockFoundation();
                    }
                    time = System.currentTimeMillis();
                    iterations = 0;
                    if (index == targets.size()) {
                        double nowTime = System.currentTimeMillis();
                        while (opModeIsActive()) {
                            robot.setDrivePower(0, 0, 0, 0);
                            telemetry.addData("Time elapsed", (nowTime - opTime) / 1000f);
                            telemetry.update();
                        }
                    }
                    target = targets.get(index);
                }
                else if(index == 1) {
                    double in1 = data.getMotorCurrentPosition(robot.in1);
                    double in2 = data.getMotorCurrentPosition(robot.in2);
                    if(Math.abs(in1 - in2) >= 500 && (Math.abs(in1) < 350 || Math.abs(in2) < 350)) {
                        requestOpModeStop();
                    }
                }
                else if(Math.abs(System.currentTimeMillis() - time) >= 5000 && ((target.getY() < 1.5 && target.getY() > -1.5) || (odometry.getY() < 1.5 && odometry.getY() > -1.5))) {
                    target.setX(target.getX() > 0 && target.getX() < 3.8 ? 4 : target.getX() > 0 && target.getX() >= 3.8 ? 3.5 : target.getX() > -3.8 ? -4 : -3.5);
                    target.setY(odometry.getY() - (0.3 * Math.signum(odometry.getY())));
                    target.setTheta(90);
                    time = System.currentTimeMillis();
                }
                else if(Math.abs(System.currentTimeMillis() - time) >= 5000 && skystone == 0 && index < 11) {
                    robot.liftbox(target.getBoxPos());
                    robot.setInPower(target.getIntakePower());
                    if(target.getDelay() != 0) {
                        robot.setDrivePower(0, 0, 0, 0);
                        double nowTime = System.currentTimeMillis();
                        while(Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                            if (target.getIntakePower() < 0.2) { // PICKING UP STONE
                                if(skystone == 2 && index < 7) {
                                    Globals.MAX_SPEED = 0.2;
                                    wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                                }
                                else {
                                    Globals.MAX_SPEED = 0.35;
                                    wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                                }
                            }
                            Globals.MAX_SPEED = 1.0;
                        }
                    }
                    index++;
                    if(index == 16) {
                        robot.unlockFoundation();
                    }
                    time = System.currentTimeMillis();
                    iterations = 0;
                    if (index != targets.size() && Math.abs(targets.get(index).getX()) > 4.5) {
                        robot.moveClamp(Clamp.MIN);
                    }
                    if (index == targets.size()) {
                        double nowTime = System.currentTimeMillis();
                        while (opModeIsActive()) {
                            robot.setDrivePower(0, 0, 0, 0);
                            telemetry.addData("Time elapsed", (nowTime - opTime) / 1000f);
                            telemetry.update();
                        }
                    }
                    target = targets.get(index);
                }
                else if(Math.abs(System.currentTimeMillis() - time) >= 5000) {
                    if(Math.abs(odometry.getX()) > 4) {
                        if(Math.signum(odometry.getX()) == -1) {
                            target.setX(odometry.getX() + 0.5);
                        }
                        else {
                            target.setX(odometry.getX() - 0.5);
                        }
                    }
                    else {
                        if(Math.signum(odometry.getX()) == -1) {
                            target.setX(odometry.getX() - 0.5);
                        }
                        else {
                            target.setX(odometry.getX() - 0.5);
                        }
                    }
                    time = System.currentTimeMillis();
                }
            }
            if(iterations > 50 && odometry.getY() > 0 && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.015) { // Move on....
                robot.liftbox(target.getBoxPos());
                robot.setInPower(target.getIntakePower());
                if(target.getDelay() != 0) {
                    robot.setDrivePower(0, 0, 0, 0);
                    double nowTime = System.currentTimeMillis();
                    while(Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                        if(target.getIntakePower() < 0.2) { // PICKING UP STONE
                            if(skystone == 2 && index < 7) {
                                Globals.MAX_SPEED = 0.2;
                                wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                            }
                            else {
                                Globals.MAX_SPEED = 0.35;
                                wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                            }                        }
                        Globals.MAX_SPEED = 1.0;
                    }
                }
                index++;
                if(index == 16) {
                    robot.unlockFoundation();
                }
                time = System.currentTimeMillis();
                iterations = 0;
                if(index != targets.size() && Math.abs(targets.get(index).getX()) > 4.5) {
                    robot.moveClamp(Clamp.MIN);
                }
                if(index == targets.size()) {
                    double nowTime = System.currentTimeMillis();
                    while(opModeIsActive()) {
                        robot.setDrivePower(0, 0, 0, 0);
                        telemetry.addData("Time elapsed", (nowTime - opTime) / 1000f);
                        telemetry.update();
                    }
                }
                target = targets.get(index);
            }
            if(index == 14 || index == 15) {
                Globals.MAX_SPEED = 0.4;
                robot.moveClamp(Clamp.MIN);
                if(index == 15) {
                    Globals.MAX_SPEED = 0.8;
                    robot.moveClamp(Clamp.MIN);
                }
            }
            else if(index == 16) {
                robot.unlockFoundation();
            }
            else if(target.getIntakePower() == -1 && Math.abs(odometry.getX()) <= 4 && index < 13) {
                Globals.MAX_SPEED = 0.35;
            }
            else {
                Globals.MAX_SPEED = 1;
            }
            wheels.update(robot, target, odometry, target.getAngle(), AngleUnit.DEGREES);
            if(target.getIntakePower() > 0) {
                robot.moveClamp(Clamp.MIN);
            }
            else if(index <= 14 && (target.getIntakePower() < 0.2 || (index + 1 < targets.size() && (targets.get(index + 1).getExtendPos() != 0 || targets.get(index + 1).getIntakePower() == -1)))) {
                robot.moveClamp(Clamp.MAX);
            }
            if(robot.ex != null) {
                robot.ex.setPower(0.5);
                robot.ex.setTargetPosition(target.getExtendPos());
                robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(target.getIntakePower() <= 0) {
                robot.setInPower(target.getIntakePower());
            }
            robot.liftbox(target.getBoxPos());
            iterations++;
            lastPoint = startPoint;
            lasttime = itertime;
        }
        else if(paused == 0) {
            robot.liftbox(target.getBoxPos());
            robot.setInPower(target.getIntakePower());
            if(target.getDelay() != 0) {
                robot.setDrivePower(0, 0, 0, 0);
                double nowTime = System.currentTimeMillis();
                while(Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                    if(target.getIntakePower() < -0.2) {
                        if(robot.hasBlock() || Math.abs(odometry.getX()) < 1.5) {
                            robot.setDrivePower(0, 0, 0, 0);
                            robot.setInPower(-0.75);
                            target.setDelay(0);
                            sleep(1000);
                            Globals.MAX_SPEED = 1.0;
                            break;
                        }
                        else {
                            if(skystone == 2 && index < 7) {
                                Globals.MAX_SPEED = 0.2;
                                wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                            }
                            else {
                                Globals.MAX_SPEED = 0.35;
                                wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                            }
                        }
                    }
                    if(target.getIntakePower() > 0.5) {
                        if(robot.hasBlock()) {
                            if(index <= 13) {
                                robot.setInPower(0.4);
                            }
                            if(Math.round(target.getDelay()) == 500 || index == 14) {
                                if(robot.mover1.getPosition() < TrashHardware.fm1Closed + 0.1) {
                                    robot.setInPower(0.4);
                                }
                                else {
                                    robot.lockFoundation();
                                    robot.setInPower(0.4);
                                }
                            }
                            else if(target.getDelay() == 501 || index == 15) {
                                robot.unlockFoundation();
                            }
                            while(robot.hasBlock()) {
                                sleep(100);
                            }
                            sleep(500);
                            if(robot.hasBlock()) {
                                while(robot.hasBlock());
                                sleep(500);
                            }
                            break;
                        }
                    }
                    if(target.getDelay() == 500.01 || index == 14) {
                        robot.lockFoundation();
                    }
                    else if(target.getDelay() == 501 || index == 15) {
                        robot.unlockFoundation();
                    }
                    Globals.MAX_SPEED = 1.0;
                }
            }
            index++;
            if(index >= 16) {
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
                    robot.setDrivePower(0, 0, 0, 0);
                    telemetry.addData("Time elapsed", nowTime - opTime);
                    telemetry.update();
                }
            }
        }
        else {
            robot.setDrivePower(0, 0, 0, 0);
            robot.ex.setPower(0);
            robot.setInPower(0);
            if(gamepad1.y) {
                paused = 0;
            }
            telemetry.addData("Angle", odometry.getAngle());
            telemetry.addData("Pose", odometry.getPoint());
            telemetry.addData("Target Angle", target.getAngle());
            telemetry.addData("Target Pose", new Point(target.getX(), target.getY()));
            telemetry.update();
            wheels.update(gamepad1, gamepad2, robot, robot.bulkRead(), robot.bulkReadTwo());
            odometry.update(gamepad1, gamepad2, robot, robot.bulkRead(), robot.bulkReadTwo());
        }
    }

    public void stopOp() {
        robot.setDrivePower(0, 0, 0, 0);
        robot.ex.setPower(0);
        robot.setInPower(0);
    }
}
