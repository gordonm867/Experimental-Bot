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

@Autonomous(name="BLUE Three Stone Autonomous", group="Trash")
@Disabled
public class BlueThreeBlock extends MyOpMode {

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

        if(odometry.getPoint().equals(new Point(-4, -2))) {
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
            if(skystone == 0) {
                skystone = 2;
            }
            else if(skystone == 2) {
                skystone = 0;
            }
            telemetry.addData("Skystone", skystone == 0 ? "Left" : skystone == 1 ? "Center" : "Right");
            telemetry.update();
        }
        skystone = 1;
    }

    public void startOp() {
        opTime = System.currentTimeMillis();
        wheels.setState(Drivetrain.State.ON);
        skystone = 1;
        if(skystone == 0) {
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-2.6, -3.7 + (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 500, true));
            targets.add(new RobotState(-3.5, -3.5, 140, 0, 0, false, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 3.25, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.15, 3.25, 180, Globals.EXTEND_POS / 5, 0.5, true, Globals.boxNeutral, 2000, true));
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, -1.15, 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true));
            targets.add(new RobotState(-2.6, -1.87 + (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, true));
            targets.add(new RobotState(-3.4, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.4, 1, 90, 0, 0.01, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.4, 4.5, 180, 0, 0.01, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3, 4.5, 180, -1090, 0.5, true, 0.41, 500, false));
            targets.add(new RobotState(-5.5, 4.5, 180, -1090, 0.5, true, 0.41, 500, true));
            targets.add(new RobotState(-5.5, 4.5, 180, -1090, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-5.5, 4.5, 180, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-5.5, 3, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 0.75, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            //targets.add(new RobotState(-3.5, 2.1, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            /*
            targets.add(new RobotState(-1.3, 0.75, 90, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-1.3, 4, 90, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-3.5, 4, 90, 0, 0, true, Globals.boxNeutral, 0, true));

             */
            targets.add(new RobotState(-3.5, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true));
        }
        else if(skystone == 1) {
            /* First Block */
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true)); // 0
            targets.add(new RobotState(-2.7, -3.4, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 5000, true)); // 1
            targets.add(new RobotState(-3.4, -2.7, 135, 0, 0, false, Globals.boxNeutral, 0, false)); // 2

            /* Place */
            targets.add(new RobotState(-3.6, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 3
            targets.add(new RobotState(-3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 4
            targets.add(new RobotState(-3.6, 3, 180, Globals.EXTEND_POS, 0.01, false, Globals.boxNeutral, 0, false)); // 5
            targets.add(new RobotState(-3.4, 3, 180, Globals.EXTEND_POS, 1, true, 0.43, 2000, true)); // 6

            /* Second Block */
            targets.add(new RobotState(-3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 7
            targets.add(new RobotState(-3.6, -1.25, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 8
            targets.add(new RobotState(-3, -1.27, 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true)); // 9
            targets.add(new RobotState(-2.5, -2, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 5000, true)); // 10

            /* Place */
            targets.add(new RobotState(-3.6, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false)); // 11
            targets.add(new RobotState(-3.6, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 12
            targets.add(new RobotState(-3.6, 4, 180, Globals.EXTEND_POS, 0, false, Globals.boxNeutral, 0, false)); // 13
            targets.add(new RobotState(-3.4, 4, 180, Globals.EXTEND_POS, 1, false, 0.43, 2000, false)); // 14

            /* Third Block */
            targets.add(new RobotState(-3.6, 1, 90, 0, 1, false, Globals.boxNeutral, 0, false)); // 15
            targets.add(new RobotState(-3.6, -1, 90, 0, 1, false, Globals.boxNeutral, 0, false)); // 16
            targets.add(new RobotState(-3, -3, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 5000, false)); // 17

            /* Place */
            targets.add(new RobotState(-3.75, -1.3, 90, 0, 0, true, Globals.boxNeutral, 0, false)); // 18
            targets.add(new RobotState(-3.75, 1, 90, 0, 0.01, true, Globals.boxNeutral, 0, false)); // 19
            targets.add(new RobotState(-3.6, 4.5, 180, 0, 0.01, false, Globals.boxNeutral, 0, false)); // 20
            targets.add(new RobotState(-2.5, 4.5, 180, 0, 1, true, 0.43, 500.01, false)); // 21

            /* Reposition */
            targets.add(new RobotState(-5.5, 4.8, 180, 0, 1, true, Globals.boxNeutral, 500.02, true)); // 22
            targets.add(new RobotState(-5.55, 2.3, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 23
            targets.add(new RobotState(-3.5, 2.3, 180, 0, 0, false, Globals.boxNeutral, 0, false)); // 24

            /* Park */
            targets.add(new RobotState(-3.5, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true)); // 26
        }
        else {
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-2.6, -3.7 - (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 500, true));
            targets.add(new RobotState(-3.5, -3.5 - (2f/3f), 140, 0, 0, false, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 3.25, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.15, 3.25, 180, Globals.EXTEND_POS / 5, 0.5, true, Globals.boxNeutral, 2000, true));
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, -1.27 - (2f/3f), 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true));
            targets.add(new RobotState(-2.6, -2 - (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, true));
            targets.add(new RobotState(-3.4, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.4, 1, 90, 0, 0.01, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.4, 4.5, 180, 0, 0.01, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3, 4.5, 180, -1090, 0.5, true, 0.41, 500, false));
            targets.add(new RobotState(-5.5, 4.5, 180, -1090, 0.5, true, 0.41, 500, true));
            targets.add(new RobotState(-5.5, 4.5, 180, -1090, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-5.5, 4.5, 180, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-5.5, 3, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 0.75, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            //targets.add(new RobotState(-3.5, 2.1, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            //targets.add(new RobotState(-3.5, 2.1, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            /*
            targets.add(new RobotState(-1.3, 0.75, 90, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-1.3, 4, 90, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-3.5, 4, 90, 0, 0, true, Globals.boxNeutral, 0, true));

             */
            targets.add(new RobotState(-3.5, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true));
        }
    }

    public void loopOp() {
        RobotState target = targets.get(index);
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
        double angular = Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()));
        if(paused == 0 && ((target.isRequired() && target.isExact() && (displacement >= 0.15 || angular >= 3 || (robot.ex != null && Math.abs(robot.ex.getCurrentPosition() - target.getExtendPos()) >= 100))) || (!target.isRequired() && !target.isExact() && (displacement >= 0.5)) || (target.isRequired() && !target.isExact() && (displacement >= 0.5 || (robot.ex != null && Math.abs(robot.ex.getCurrentPosition() - target.getExtendPos()) >= 100))) || (!target.isRequired() && target.isExact() && (displacement >= 0.15 || angular >= 3)))) {
            if(gamepad1.x) {
                paused = 1;
            }
            Point startPoint = odometry.getPoint();
            double itertime = System.currentTimeMillis();
            if(System.currentTimeMillis() - time >= 3000 && ((target.getY() < 0 && target.getY() > -2) || (odometry.getY() < 0 && odometry.getY() > -2))) {
                if(skystone == 0 && index < 11) {
                    robot.liftbox(target.getBoxPos());
                    robot.setInPower(target.getIntakePower());
                    if (target.getDelay() != 0) {
                        robot.setDrivePower(0, 0, 0, 0);
                        double nowTime = System.currentTimeMillis();
                        while (Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                            if (target.getIntakePower() < 0.2) { // PICKING UP STONE
                                Globals.MAX_SPEED = 0.35;
                                wheels.update(robot, new Point(odometry.getX() + (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                            }
                            Globals.MAX_SPEED = 1.0;
                        }
                    }
                    index++;
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
                else {
                    target.setX(target.getX() > 0 && target.getX() < 3.8 ? 4 : target.getX() > 0 && target.getX() >= 3.8 ? 3.75 : target.getX() > -3.8 ? -4 : -3.75);
                    target.setY(odometry.getY() - 0.3);
                    target.setTheta(90);
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
                            Globals.MAX_SPEED = 0.35;
                            wheels.update(robot, new Point(odometry.getX() + (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                        }
                        Globals.MAX_SPEED = 1.0;
                    }
                }
                index++;
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
            if(index == 21 || index == 22) {
                Globals.MAX_SPEED = 0.75;
                if(index == 22) {
                    robot.moveClamp(Clamp.MIN);
                }
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
            else if(index <= 19 && (target.getIntakePower() < 0.2 || (index + 1 < targets.size() && (targets.get(index + 1).getExtendPos() != 0 || targets.get(index + 1).getIntakePower() == -1)))) {
                robot.moveClamp(Clamp.MAX);
            }
            if(robot.ex != null) {
                robot.ex.setPower(0.75);
                robot.ex.setTargetPosition(target.getExtendPos());
                robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(target.getIntakePower() <= 0) {
                robot.setInPower(target.getIntakePower());
            }
            /*
            if((index == 17 || index == 18) && robot.hasBlock()) {
                sleep(200);
                if(robot.hasBlock()) {
                    robot.setInPower(-0.75);
                    sleep(1000);
                    index++;
                    if (index == 18) {
                        index++;
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
            } */
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
                        if(robot.hasBlock() || Math.abs(odometry.getX()) < 0.5) {
                            robot.setDrivePower(0, 0, 0, 0);
                            robot.setInPower(-0.75);
                            target.setDelay(0);
                            sleep(1000);
                            Globals.MAX_SPEED = 1.0;
                            break;
                        }
                        else {
                            Globals.MAX_SPEED = 0.35;
                            wheels.update(robot, new Point(odometry.getX() + 0.5, odometry.getY() - 0.5), odometry, odometry.getAngle(), AngleUnit.DEGREES);
                        }
                    }
                    if(target.getIntakePower() > 0.5) {
                        if(robot.hasBlock()) {
                            robot.setInPower(0.4);
                            if(target.getDelay() == 500.01 || index == 21) {
                                robot.lockFoundation();
                            }
                            else if(target.getDelay() == 500.02 || index == 22) {
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
                    if(target.getDelay() == 500.01 || index == 21) {
                        robot.lockFoundation();
                    }
                    else if(target.getDelay() == 500.02 || index == 22) {
                        robot.unlockFoundation();
                    }
                    Globals.MAX_SPEED = 1.0;
                }
            }
            index++;
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
