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

@Autonomous(name="RED Three Stone Autonomous", group="Trash")
@Disabled
public class RedThreeBlock extends MyOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);

    public static int skystone = 1;

    Point target;
    Point lastPoint;

    public static double TARGET_THETA = 0;
    public static int READY = 0;
    public static int index = 0;
    public static int paused = 0;

    private int iterations = 0;

    double lasttime = System.currentTimeMillis();

    ArrayList<RobotState> targets = new ArrayList<>();

    private OpenCvCamera phoneCam;


    private double opTime = System.currentTimeMillis();

    public void initOp() {
        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.moveClamp(Clamp.MIN);
        robot.liftbox(Globals.boxNeutral);
        robot.unlockFoundation();
        Globals.boxDown = 0.395;
        Globals.START_X = 5.25;
        Globals.START_THETA = 0;

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
            READY = 0;
        }
        Globals.MAX_SPEED = 1.0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        DetectionPipeline pipeline = new DetectionPipeline();
        phoneCam.setPipeline(pipeline);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        while(!pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }
        while(!isStarted() && !isStopRequested()) {
            robot.setDrivePower(0, 0, 0, 0);
            telemetry.addData("Status", "Initialized");
            skystone = pipeline.getBestCol();
            telemetry.addData("Skystone", skystone == 0 ? "Left" : skystone == 1 ? "Center" : "Right");
            telemetry.update();
        }

        if(skystone == 0) {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(2.6, -3.7 + (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, true));
            targets.add(new RobotState(3.5, -3.5 + (2f/3f), 140, 0, 0, false, Globals.boxNeutral, 0, true));

            /* Place */
            targets.add(new RobotState(3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.5, 3.25, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, 3.25, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 1000, true));

            /* Second Block */
            targets.add(new RobotState(3, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1.3 + (2f/3f), 90, Globals.EXTEND_POS, 0, false, Globals.boxDown, 0, false));
            targets.add(new RobotState(1.8, -1.3 + (2f/3f), 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, false));

            /* Place */
            targets.add(new RobotState(3.4, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 3.875, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, 3.875, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 1000, true));

            /* Third Block */
            targets.add(new RobotState(3, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1.3, 90, Globals.EXTEND_POS, 0, false, Globals.boxDown, 0, false));
            targets.add(new RobotState(1.8, -1.3, 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, false));


            /* Place */
            targets.add(new RobotState(3.4, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 4.5, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(2.4, 4.5, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 1000, false));

            /* Reposition */
            targets.add(new RobotState(5.2, 4.5, 180, 0, 0, true, 0.425, 500, true));
            targets.add(new RobotState(5.25, 3, 180, 0, 0, false, 0.425, 0, false));
            targets.add(new RobotState(3.25, 0.75, 90, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(3.25, 2.1, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true));

            /* Park */
            targets.add(new RobotState(3.25, 0, 90, Globals.EXTEND_POS, 0, true, Globals.boxNeutral, 0, true));
        }
        else if(skystone == 1) {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(2.6, -3.7, 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, true));
            targets.add(new RobotState(3.5, -3.5, 140, 0, 0, false, Globals.boxNeutral, 0, true));

            /* Place */
            targets.add(new RobotState(3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.5, 3.25, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, 3.25, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 1000, true));

            /* Second block */
            targets.add(new RobotState(3, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1.3, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(1.8, -1.3, 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, false));

            /* Place */
            targets.add(new RobotState(3.4, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 3.875, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, 3.875, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 1000, false));

            /* Third Block */
            targets.add(new RobotState(3, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1.3 - (2f/3f), 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(1.8, -1.3 - (2f/3f), 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, false));

            /* Place */
            targets.add(new RobotState(3.4, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 4.5, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(2.4, 4.5, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 1000, false));

            /* Reposition */
            targets.add(new RobotState(5.2, 4.5, 180, 0, 0, true, 0.425, 500, true));
            targets.add(new RobotState(5.25, 3, 180, 0, 0, false, 0.425, 0, false));
            targets.add(new RobotState(3.25, 0.75, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(3.25, 2.1, 180, 0, 0, true, Globals.boxNeutral, 0, true));

            /* Park */
            targets.add(new RobotState(3.25, 0, 180, 0, 0, true, Globals.boxNeutral, 0, true));
        }
        else {
            /* First Block */
            targets.add(new RobotState(5.25, -3.2, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(2.6, -3.7 - (2f/3f), 135, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, true));
            targets.add(new RobotState(3.5, -3.5 - (2f/3f), 140, 0, 0, false, Globals.boxNeutral, 0, true));

            /* Place */
            targets.add(new RobotState(3.5, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.5, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.5, 3.25, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, 3.25, 180, Globals.EXTEND_POS, 1, true, Globals.boxNeutral, 1000, true));

            /* Second block */
            targets.add(new RobotState(3, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1.3 - (2f/3f), 90, Globals.EXTEND_POS, 0, false, Globals.boxDown, 0, false));
            targets.add(new RobotState(1.8, -1.3 - (2f/3f), 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, false));

            /* Place */
            targets.add(new RobotState(3.4, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 3.875, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, 3.875, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 1000, false));

            /* Third Block */
            targets.add(new RobotState(3, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3, -1.3 - (4f/3f), 90, Globals.EXTEND_POS, 0, false, Globals.boxDown, 0, false));
            targets.add(new RobotState(1.8, -1.3 - (4f/3f), 90, Globals.EXTEND_POS, -1, true, Globals.boxDown, 1000, false));

            /* Place */
            targets.add(new RobotState(3.4, -1, 90, 0, 1, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 1, 90, 0, 0, true, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(3.4, 4.5, 180, 0, 0, false, Globals.boxNeutral, 0, false));
            targets.add(new RobotState(2.4, 4.5, 180, Globals.EXTEND_POS, 1, false, Globals.boxNeutral, 1000, false));

            /* Reposition */
            targets.add(new RobotState(5.2, 4.5, 180, 0, 0, true, 0.425, 500, true));
            targets.add(new RobotState(5.25, 3, 180, 0, 0, false, 0.425, 0, false));
            targets.add(new RobotState(3.25, 0.75, 180, 0, 0, true, Globals.boxNeutral, 0, true));
            targets.add(new RobotState(3.25, 2.1, 180, 0, 0, true, Globals.boxNeutral, 0, true));

            /* Park */
            targets.add(new RobotState(3.25, 0, 180, 0, 0, true, Globals.boxNeutral, 0, true));

        }
    }

    public void startOp() {
        wheels.setState(Drivetrain.State.ON);
    }

    public void loopOp() {
        RobotState target = targets.get(index);
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
        double angular = Functions.normalize(Math.abs(odometry.getAngle() - target.getAngle()));
        if(paused == 0 && ((target.isRequired() && target.isExact() && (displacement >= 0.15 || angular >= 1.5 || (robot.ex != null && Math.abs(robot.ex.getCurrentPosition() - target.getExtendPos()) >= 100))) || (!target.isRequired() && !target.isExact() && (displacement >= 0.5)) || (target.isRequired() && !target.isExact() && (displacement >= 0.5 || (robot.ex != null && Math.abs(robot.ex.getCurrentPosition() - target.getExtendPos()) >= 100))) || (!target.isRequired() && target.isExact() && (displacement >= 0.15 || angular >= 1.5)))) {
            Point startPoint = odometry.getPoint();
            double itertime = System.currentTimeMillis();
            if(iterations > 20 && Math.abs((startPoint.distance(lastPoint) * -Globals.DRIVE_FEET_PER_TICK) / ((lasttime - itertime) / 1000.0)) <= 0.015) {
                index++;
                target = targets.get(index);
                iterations = 0;
            }
            if(index == 14) {
                Globals.MAX_SPEED = 0.75;
            }
            if(index == 15) {
                Globals.MAX_SPEED = 1;
            }
            if(skystone == 2 && index == 1 && Math.abs(odometry.getX()) <= 3.5) {
                Globals.MAX_SPEED = 0.3;
            }
            else if(skystone == 2) {
                Globals.MAX_SPEED = 1;
            }
            wheels.update(robot, target, odometry, Functions.normalize(target.getAngle() - 180), AngleUnit.DEGREES);
            if(index != targets.size() - 1 && Math.abs(odometry.getY()) <= 0.5 && target.getIntakePower() == 0) {
                robot.setInPower(1);
            }
            else if(target.getIntakePower() == 0) {
                robot.setInPower(0);
            }
            if(target.getIntakePower() == 1) {
                robot.moveClamp(Clamp.MIN);
            }
            robot.liftbox(target.getBoxPos());
            if(robot.ex != null) {
                robot.ex.setPower(0.75);
                robot.ex.setTargetPosition(target.getExtendPos());
                robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(target.getIntakePower() <= 0) {
                robot.moveClamp(0.125);
                if(!(index != targets.size() - 1 && Math.abs(odometry.getY()) <= 0.2)) {
                    robot.setInPower(target.getIntakePower());
                }
            }
            iterations++;
            lastPoint = startPoint;
            lasttime = itertime;
        }
        else if(paused == 0) {
            robot.setInPower(target.getIntakePower());
            if(target.getDelay() != 0) {
                robot.setDrivePower(0, 0, 0, 0);
                double nowTime = System.currentTimeMillis();
                while(Math.abs(System.currentTimeMillis() - nowTime) <= target.getDelay()) {
                    if(target.getIntakePower() == -1 && index >= 6 && index <= 15) { // PICKING UP FIRST STONE
                        Globals.MAX_SPEED = 0.3;
                        wheels.update(robot, new Point(odometry.getX() + (0.5 * Math.cos(Math.toRadians(odometry.getAngle()))), odometry.getY() - (0.5 * Math.sin(Math.toRadians(odometry.getAngle())))), odometry, 150, AngleUnit.DEGREES);
                    }
                    else if(target.getIntakePower() == -1 && index <= 15) { // SECOND STONE
                        Globals.MAX_SPEED = 0.15;
                        wheels.update(robot, new Point(odometry.getX(), odometry.getY() - 0.5), odometry, 90, AngleUnit.DEGREES);
                    }
                    else if(target.getIntakePower() == -1) { // THIRD STONE
                        Globals.MAX_SPEED = 0.15;
                        wheels.update(robot, new Point(odometry.getX(), odometry.getY() - 0.5), odometry, 90, AngleUnit.DEGREES);
                    }
                    else if(target.getIntakePower() == 1 && index >= 22) { // MOVING FOUNDATION
                        Globals.MAX_SPEED = 0.15;
                        wheels.update(robot, new Point(odometry.getX() - 0.5, odometry.getY()), odometry, target.getAngle(), AngleUnit.DEGREES);
                        if(System.currentTimeMillis() - nowTime <= 750) {
                            robot.liftbox(Globals.boxNeutral);
                        }
                        if(System.currentTimeMillis() - nowTime <= 500) {
                            robot.lockFoundation();
                        }
                    }
                    else if(target.getIntakePower() == 1) { // RELEASING NORMALLY
                        if(System.currentTimeMillis() - nowTime <= 750) {
                            robot.liftbox(Globals.boxNeutral);
                        }
                    }
                    else if(index >= 22){ // RELEASING FOUNDATION
                        Globals.MAX_SPEED = 0.5;
                        wheels.update(robot, new Point(odometry.getX() + 0.5, odometry.getY() + 0.5), odometry, target.getAngle(), AngleUnit.DEGREES);
                        robot.unlockFoundation();
                    }
                    Globals.MAX_SPEED = 1.0;
                }
            }
            index++;
            iterations = 0;
            if(Math.abs(targets.get(index).getX()) > 4.5) {
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

        }
    }

    public void stopOp() {
        robot.setDrivePower(0, 0, 0, 0);
        robot.ex.setPower(0);
        robot.setInPower(0);
    }
}
