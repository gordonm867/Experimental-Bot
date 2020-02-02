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

@Autonomous(name="RED 1st Foundation Two Stone Autonomous", group="Trash")
@Config
//@Disabled
public class RedTwoBlockFoundFirst extends MyOpMode {

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
            while(opModeIsActive() && ((displacement >= 0.25 || (Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()))) >= 1) && READY != 0)) {
                RevBulkData data = robot.bulkRead();
                odometry.update(data);
                displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
                TARGET_THETA = target.getAngle();
                if(Double.isNaN(target.getAngle())) {
                    TARGET_THETA = odometry.getAngle();
                }
                if((displacement >= 0.25 || (Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()))) >= 1) && READY != 0) {
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
            targets.add(new RobotState(2.7, -3.15 + (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 1
            targets.add(new RobotState(3.3, -3.15 + (2f / 3f), 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(3.6, 4, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(2.8, 4, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 6

            /* Reposition */
            targets.add(new RobotState(5.25, 4, 180, 0, 1, true, Globals.boxNeutral, 501, true)); // 7
            targets.add(new RobotState(5.25, 1.75, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 8

            /* Second Block */
            //targets.add(new RobotState(3.6, -1.25 + (2f / 3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(3.6, 1.75, 90, Globals.EXTEND_POS, 0, false, Globals.boxNeutral , 0, false)); // 9
            targets.add(new RobotState(3.6, -1, 90, Globals.EXTEND_POS, 0, false, Globals.boxNeutral , 0, false)); // 9
            targets.add(new RobotState(3.6, -2 + (2f / 3f), 135, Globals.EXTEND_POS, 0, false, Globals.boxNeutral, 0, true)); // 9
            targets.add(new RobotState(2.7, -1.7 + (2f / 3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 10

            /* Place */
            targets.add(new RobotState(3.6, -1, -90, 0, 1, true, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(3.6, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(3.8, 2.25, -80, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 2000, true)); // 13

            targets.add(new RobotState(3.6, -1, 100, 0, 0, true, Globals.boxNeutral, 0, false)); // 14
            targets.add(new RobotState(2.7, -3, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 15
            targets.add(new RobotState(2.5, -3, 135, 0, 0, true, Globals.boxNeutral, 0, true)); // 16
            targets.add(new RobotState(3.8, 2.05, -100, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 17
            targets.add(new RobotState(3.8, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 18

        }
        else if(skystone == 1) {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(2.7, -3.4, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 1
            targets.add(new RobotState(3.3, -3.4, 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(3.6, 4, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(2.8, 4, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 6

            /* Reposition */
            targets.add(new RobotState(5.25, 4, 180, 0, 1, true, Globals.boxNeutral, 501, true)); // 7
            targets.add(new RobotState(5.25, 1.75, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 8

            /* Second Block */
            //targets.add(new RobotState(3.6, -1.25 + (2f / 3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(3.3, 1.75, 90, Globals.EXTEND_POS, 0, false, Globals.boxNeutral , 0, false)); // 9
            targets.add(new RobotState(3.3, -1, 90, Globals.EXTEND_POS, 0, false, Globals.boxNeutral , 0, false)); // 10
            targets.add(new RobotState(3.3, -2.8, 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true)); // 11
            targets.add(new RobotState(2.7, -2.8, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 12

            /* Place */
            targets.add(new RobotState(3.6, -1, -90, 0, 1, true, Globals.boxNeutral, 0, false)); // 13
            targets.add(new RobotState(3.6, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 14
            targets.add(new RobotState(3.8, 2.25, -80, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 2000, true)); // 15

            targets.add(new RobotState(3.3, -1, 100, 0, 0, true, Globals.boxNeutral, 0, false)); // 16
            targets.add(new RobotState(2.7, -3, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 17
            targets.add(new RobotState(2.5, -3, 135, 0, 0, true, Globals.boxNeutral, 0, true)); // 18
            targets.add(new RobotState(3.4, 2.05, -100, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 19
            targets.add(new RobotState(3.4, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 20
        }
        else {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(2.7, -3.4 + (4f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 1
            targets.add(new RobotState(3.4, -3.4 + (4f/3f), 90, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(3.6, 4, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(2.8, 4, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 6

            /* Reposition */
            targets.add(new RobotState(5.35, 4, 180, 0, 1, true, Globals.boxNeutral, 501, true)); // 7
            targets.add(new RobotState(5.35, 1.75, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 8

            /* Second Block */
            //targets.add(new RobotState(3.6, -1.25 + (2f / 3f), 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(3.3, -1, 90, Globals.EXTEND_POS, 0, false, Globals.boxNeutral , 0, true)); // 9
            targets.add(new RobotState(2.5, -3 - (2f/3f), 90, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true)); // 9
            targets.add(new RobotState(2, -3 - (2f/3f), 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 10

            /* Place */
            targets.add(new RobotState(3.6, -1, -90, 0, 1, true, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(3.6, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(3.8, 2.25, -80, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 2000, true)); // 13

            targets.add(new RobotState(3.3, -1, 100, 0, 0, true, Globals.boxNeutral, 0, false)); // 14
            targets.add(new RobotState(2.7, -3, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 3000, true)); // 15
            targets.add(new RobotState(2.5, -3, 135, 0, 0, true, Globals.boxNeutral, 0, true)); // 16
            targets.add(new RobotState(3.4, 2.05, -100, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 2000, true)); // 17
            targets.add(new RobotState(3.4, 1, -90, 0, 0, true, Globals.boxNeutral, 0, false)); // 18
        }
        for(RobotState target : targets) {
            target.setTheta(180 - target.getAngle());
        }
    }

    public void loopOp() {
        if(index >= 8) {
            robot.unlockFoundation();
        }
        RobotState target = targets.get(index);
        if(index == 2 || (skystone == 0 && index == 16)) {
            target.setY(odometry.getY());
        }
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
        double angular = Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()));
        RevBulkData data = robot.bulkRead();
        int expos = data.getMotorCurrentPosition(robot.ex);
        if(paused == 0 && ((target.isRequired() && target.isExact() && (displacement >= 0.25 || angular >= 3 || (robot.ex != null && Math.abs(expos - target.getExtendPos()) >= 300))) || (!target.isRequired() && !target.isExact() && (displacement >= 0.5)) || (target.isRequired() && !target.isExact() && (displacement >= 0.5 || (robot.ex != null && Math.abs(expos - target.getExtendPos()) >= 300))) || (!target.isRequired() && target.isExact() && (displacement >= 0.25 || angular >= 3)))) {
            if(Math.abs(opTime - System.currentTimeMillis()) >= 28000) {
                robot.liftbox(Globals.boxNeutral);
                robot.ex.setTargetPosition(0);
                robot.ex.setPower(0.5);
                target.setX(3.3);
                target.setY(0);
                target.setTheta(90 * Math.signum(odometry.getAngle()));
                while(opModeIsActive() && Math.abs(odometry.getY()) > 0.5) {
                    wheels.update(robot, target, odometry, target.getAngle(), AngleUnit.DEGREES);
                }
                while(opModeIsActive()) {
                    double nTime = System.currentTimeMillis();
                    if(Math.abs(odometry.getX()) < 4) {
                        robot.moveClamp(Clamp.MAX);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    telemetry.addData("Time elapsed", (nTime - opTime) / 1000f);
                    telemetry.update();
                }
            }
            if(target.getIntakePower() < 0.1) {
                if(robot.hasBlock()) {
                    index++;
                    if (index == targets.size()) {
                        double nowTime = System.currentTimeMillis();
                        while (opModeIsActive()) {
                            if(Math.abs(odometry.getX()) < 4) {
                                robot.moveClamp(Clamp.MAX);
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            telemetry.addData("Time elapsed", (nowTime - opTime) / 1000f);
                            telemetry.update();
                        }
                    }
                    target = targets.get(index);
                    time = System.currentTimeMillis();
                }
                else {
                    if(Math.abs(odometry.getX()) < 4) {
                        robot.moveClamp(Clamp.MAX);
                    }
                }
            }
            if(index > 1 && targets.get(index - 1).getIntakePower() < 0.2) {
                robot.setInPower(-0.75);
            }
            if(gamepad1.x) {
                paused = 1;
            }
            Point startPoint = odometry.getPoint();
            double itertime = System.currentTimeMillis();
            if(iterations > 50 && odometry.getY() > 0 && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.3) { // Move on....
                telemetry.addData("Oh no", "I'm stuck!");
                telemetry.update();
                if(!(Math.abs(target.getX()) > 5)) {
                    if (Math.abs(odometry.getX()) < 3.3) {
                        target.setX(3.3 + (0.5 * Math.signum(odometry.getX())));
                    } else {
                        target.setX(3.8 - (0.5 * Math.signum(odometry.getX())));
                    }
                    if (odometry.getY() > 1) {
                        target.setY(2);
                    } else {
                        target.setY(-2);
                    }
                    target.setTheta(90 * Math.signum(odometry.getAngle()));
                    while (opModeIsActive() && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.15 && Math.abs(opTime - System.currentTimeMillis()) <= 28000) {
                        wheels.update(robot, target, odometry, target.getAngle(), AngleUnit.DEGREES);
                    }
                }
                index++;
                if(index >= 8) {
                    robot.unlockFoundation();
                }
                time = System.currentTimeMillis();
                iterations = 0;
                target = targets.get(index);
            }
            if(index == 6 || index == 7) {
                Globals.MAX_SPEED = 0.35;
                robot.moveClamp(Clamp.MIN);
                if(index == 7) {
                    robot.lockFoundation();
                    robot.moveClamp(Clamp.MIN);
                    Globals.MAX_SPEED = 0.8;
                    robot.moveClamp(Clamp.MIN);
                }
            }
            else if(index == 8) {
                robot.unlockFoundation();
            }
            else if((index == 15 && skystone != 2) || (index == 14 && skystone == 2)) {
                if(odometry.getY() > 2) {
                    Globals.MAX_SPEED = 0.3;
                }
            }
            else if(skystone == 2 && (index == 10 || index == 11)) {
                if(odometry.getY() > 1) {
                    Globals.MAX_SPEED = 0.3;
                }
            }
            else if((index == 17 && skystone != 2) || (index == 16 && skystone == 2)) {
                if(odometry.getY() > 0.5) {
                    Globals.MAX_SPEED = 0.3;
                }
            }
            else if((index == 11 || index == 12) && ((skystone == 1 && odometry.getY() < -1.3) || skystone == 0 && odometry.getY() < 0.1)) {
                Globals.MAX_SPEED = 0.3;
            }
            else if(target.getIntakePower() == -1 && Math.abs(odometry.getX()) <= 4 && index < 11) {
                Globals.MAX_SPEED = 0.35;
            }
            else {
                Globals.MAX_SPEED = 1;
            }
            wheels.update(robot, target, odometry, target.getAngle(), AngleUnit.DEGREES);
            if(target.getIntakePower() > 0) {
                robot.moveClamp(Clamp.MIN);
            }
            else if(index >= 9 && (target.getIntakePower() < 0.2 || (index + 1 < targets.size() && (targets.get(index + 1).getExtendPos() != 0 || targets.get(index + 1).getIntakePower() == -1)))) {
                if(Math.abs(odometry.getX()) < 4) {
                    robot.moveClamp(Clamp.MAX);
                }
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
                while(opModeIsActive() && Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                    if(Math.abs(opTime - System.currentTimeMillis()) >= 28000) {
                        robot.liftbox(Globals.boxNeutral);
                        robot.ex.setTargetPosition(0);
                        robot.ex.setPower(0.5);
                        target.setX(3.3);
                        target.setY(0);
                        target.setTheta(90 * Math.signum(odometry.getAngle()));
                        while(opModeIsActive() && Math.abs(odometry.getY()) > 0.5) {
                            wheels.update(robot, target, odometry, target.getAngle(), AngleUnit.DEGREES);
                        }
                        while(opModeIsActive()) {
                            double nTime = System.currentTimeMillis();
                            if(Math.abs(odometry.getX()) < 4) {
                                robot.moveClamp(Clamp.MAX);
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
                            if(Math.abs(odometry.getX()) < 4) {
                                robot.moveClamp(Clamp.MAX);
                            }
                            Globals.MAX_SPEED = 0.3;
                            wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                        }
                    }
                    if(target.getIntakePower() > 0.5) {
                        if(robot.hasBlock()) {
                            robot.setInPower(0.33);
                            if(index < 10) {
                                robot.lockFoundation();
                                robot.moveClamp(Clamp.MIN);
                                break;
                            }
                            while(opModeIsActive() && robot.hasBlock()) {
                                sleep(100);
                            }
                            sleep(500);
                            if(robot.hasBlock()) {
                                while(opModeIsActive() && robot.hasBlock());
                                sleep(500);
                            }
                            break;
                        }
                        else if(index < 10) {
                            break;
                        }
                    }
                    if(index == 6) {
                        robot.lockFoundation();
                        robot.moveClamp(Clamp.MIN);
                    }
                    else if(index == 7) {
                        robot.unlockFoundation();
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
                    if(Math.abs(odometry.getX()) < 4) {
                        robot.moveClamp(Clamp.MAX);
                    }
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










            /*
            if(Math.abs(System.currentTimeMillis() - time) >= 2000) {
                if(target.getDelay() == 0 && target.isRequired() && ((target.isExact() && displacement < 0.25) || (!target.isExact() && displacement < 0.5))) {
                    index++;
                    if(index == 8) {
                        robot.unlockFoundation();
                    }
                    if(index == 7) {
                        robot.lockFoundation();
                        robot.moveClamp(Clamp.MIN);
                    }
                    time = System.currentTimeMillis();
                    iterations = 0;
                    if (index == targets.size()) {
                        double nowTime = System.currentTimeMillis();
                        while (opModeIsActive()) {
                            if(Math.abs(odometry.getX()) < 4) {
                                robot.moveClamp(Clamp.MAX);
                            }
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
                else if(Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.015 && Math.abs(System.currentTimeMillis() - time) >= 2500 && ((target.getY() < 1.5 && target.getY() > -1.5) || (odometry.getY() < 1.5 && odometry.getY() > -1.5))) {
                    target.setX(target.getX() > 0 && target.getX() < 3.8 ? odometry.getX() + 0.3 : target.getX() > 0 && target.getX() >= 3.8 ? 3.5 : target.getX() > -3.8 ? -4 : -3.5);
                    target.setExact(false);
                    Globals.MAX_SPEED = 1;
                    double start = odometry.getX();
                    if(target.getX() == 6) {
                        while(opModeIsActive() && Math.abs(start - odometry.getX()) < 0.2) {
                            wheels.update(robot, target, odometry, odometry.getAngle(), AngleUnit.DEGREES);
                        }
                    }
                    time = System.currentTimeMillis();
                }
                else if(Math.abs(System.currentTimeMillis() - time) >= 5000 && skystone == 0 && index < 13) {
                    robot.liftbox(target.getBoxPos());
                    robot.setInPower(target.getIntakePower());
                    if(target.getDelay() != 0) {
                        robot.setDrivePower(0, 0, 0, 0);
                        double nowTime = System.currentTimeMillis();
                        while(opModeIsActive() && Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                            if(target.getIntakePower() < 0.2) { // PICKING UP STONE
                                if(Math.abs(odometry.getX()) < 4) {
                                    robot.moveClamp(Clamp.MAX);
                                }
                                Globals.MAX_SPEED = 0.3;
                                wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                            }
                            Globals.MAX_SPEED = 1.0;
                        }
                    }
                    index++;
                    if(index == 8) {
                        robot.unlockFoundation();
                    }
                    if(index == 7) {
                        robot.lockFoundation();
                        robot.moveClamp(Clamp.MIN);
                    }
                    time = System.currentTimeMillis();
                    iterations = 0;
                    if (index != targets.size() && Math.abs(targets.get(index).getX()) > 4.5) {
                        robot.moveClamp(Clamp.MIN);
                    }
                    if (index == targets.size()) {
                        double nowTime = System.currentTimeMillis();
                        while (opModeIsActive()) {
                            if(Math.abs(odometry.getX()) < 4) {
                                robot.moveClamp(Clamp.MAX);
                            }
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
            */