package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@Autonomous(name="BLUE Old Two Stone Autonomous", group="Trash")
@Disabled
//@Config
public class BlueTwoBlock extends MyOpMode {

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
    }

    public void startOp() {
        wheels.setState(Drivetrain.State.ON);
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
            targets.add(new RobotState(-5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-2.6, -3.7, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 500, true));
            targets.add(new RobotState(-3.5, -3.5, 140, 0, 0, false, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, 3.25, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.15, 3.25, 180, Globals.EXTEND_POS / 5, 0.5, true, Globals.boxNeutral, 2000, true));
            targets.add(new RobotState(-3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-3, -1.27, 135, Globals.EXTEND_POS, 0, false, Globals.boxDown , 0, true));
            targets.add(new RobotState(-2.6, -2, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, true));
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
            targets.add(new RobotState(-1.3, 0.75, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(-1.3, 4, 90, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(-3.5, 4, 90, 0, 0, true, Globals.boxNeutral, 0, true));
             */
            targets.add(new RobotState(-3.5, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true));
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
            if(skystone == 0 && System.currentTimeMillis() - time >= 3000 && target.getY() < 0 && target.getY() > -2) {
                robot.liftbox(target.getBoxPos());
                robot.setInPower(target.getIntakePower());
                if(target.getDelay() != 0) {
                    robot.setDrivePower(0, 0, 0, 0);
                    double nowTime = System.currentTimeMillis();
                    while(Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                        if(target.getIntakePower() < 0.2) { // PICKING UP STONE
                            Globals.MAX_SPEED = 0.3;
                            wheels.update(robot, new Point(odometry.getX() + (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, 150, AngleUnit.DEGREES);
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
                        telemetry.addData("Time elapsed", nowTime - opTime);
                        telemetry.update();
                    }
                }
                target = targets.get(index);
            }
            if(iterations > 50 && odometry.getY() > 0 && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.015) { // Move on....
                robot.liftbox(target.getBoxPos());
                robot.setInPower(target.getIntakePower());
                if(target.getDelay() != 0) {
                    robot.setDrivePower(0, 0, 0, 0);
                    double nowTime = System.currentTimeMillis();
                    while(Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                        if(target.getIntakePower() < 0.2) { // PICKING UP STONE
                            Globals.MAX_SPEED = 0.3;
                            wheels.update(robot, new Point(odometry.getX() + (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, 150, AngleUnit.DEGREES);
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
                        telemetry.addData("Time elapsed", nowTime - opTime);
                        telemetry.update();
                    }
                }
                target = targets.get(index);
            }
            if(index > 1 && targets.get(index - 1).getIntakePower() == -1 && target.getIntakePower() == 0) {
                robot.setInPower(0.25);
            }
            else {
                if(target.getIntakePower() == 0) {
                    robot.setInPower(0);
                }
            }
            if(index == 14 || index == 15) {
                Globals.MAX_SPEED = 0.75;
                if(index == 15) {
                    Globals.MAX_SPEED = 0.5;
                    robot.moveClamp(Clamp.MIN);
                }
            }
            else if(target.getIntakePower() == -1 && Math.abs(odometry.getX()) <= 4) {
                Globals.MAX_SPEED = 0.4;
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
            if(!(target.getY() > 0 && Math.round(Math.abs(target.getX())) != 3 && Math.round(Math.abs(target.getX())) != 4)) {
                robot.liftbox(target.getBoxPos());
            }
            if(robot.ex != null) {
                robot.ex.setPower(0.75);
                robot.ex.setTargetPosition(target.getExtendPos());
                robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(target.getIntakePower() <= 0) {
                robot.setInPower(target.getIntakePower());
            }
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
                    if(target.getIntakePower() < 0.2 && (index >= 6 || skystone == 2)) { // PICKING UP STONE
                        Globals.MAX_SPEED = 0.3;
                        wheels.update(robot, new Point(odometry.getX() - (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, 150, AngleUnit.DEGREES);
                    }
                    else if(target.getIntakePower() < 0.2) {
                        Globals.MAX_SPEED = 0.25;
                        wheels.update(robot, new Point(odometry.getX() + 0.5, odometry.getY() - 0.5), odometry, 135, AngleUnit.DEGREES);
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
