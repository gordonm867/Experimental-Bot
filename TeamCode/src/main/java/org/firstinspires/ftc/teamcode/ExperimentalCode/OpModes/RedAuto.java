package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Circle;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Line;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name="RED Auto", group="Trash")
public class RedAuto extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);
    private double angleOffset = 3;
    private double radius = 1.86;
    private double turnCoeff = 0.0055 ;
    private boolean test = true;
    private boolean fastturns = false;
    private double starttime = 0;

    private OpenCvCamera camera;
    private int skystone = 1;

    public void runOpMode() throws InterruptedException {

        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        DetectionPipeline pipeline = new DetectionPipeline();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        while(!pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }

        /* Select Configuration */
        boolean changed = false;
        ArrayList<String> pathNames = new ArrayList<>();
        pathNames.add("Skystone RED Solo Auto");
        pathNames.add("Skystone RED Loading Auto");
        pathNames.add("Skystone RED Building Auto");
        pathNames.add("Skystone RED Meet One Auto");
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

        /* Initialize Subsystems */
        if(pathNum % pathNames.size() == 0) {
            Globals.START_X = 4.7;
            Globals.START_Y = -3;
            Globals.START_THETA = 180;
        }
        else if(pathNum % pathNames.size() == 1) {
            Globals.START_X = 4.7;
            Globals.START_Y = -3;
            Globals.START_THETA = 180;
        }
        else if(pathNum % pathNames.size() == 2) {
            Globals.START_X = 4.7;
            Globals.START_Y = -3;
            Globals.START_THETA = 180;
        }
        else if(pathNum % pathNames.size() == 3) {
            Globals.START_X = 5;
            Globals.START_Y = -3;
            Globals.START_THETA = 180;
        }
        odometry = Odometry.getInstance(robot);
        odometry.reset();
        robot.enabled = true;
        while(!isStarted() && !isStopRequested()) {
            skystone = pipeline.getBestCol();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Skystone", skystone == 0 ? "Left" : skystone == 1 ? "Center" : "Right");
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Mode", pathNames.get(pathNum % pathNames.size()));
            telemetry.update();
        }
        skystone++;
        waitForStart();
        //camera.closeCameraDevice();
        starttime = System.currentTimeMillis();
        wheels.setState(Drivetrain.State.ON);

        ArrayList<Line> path = new ArrayList<>();
        if(pathNum % pathNames.size() == 0) {
            radius = 0.75;
            turnCoeff = 0.0055 ;
            if(skystone == 1) {
                turnToPoint(new Point(4.25, -3.51));
                path.add(new Line(odometry.getPoint(), new Point(4.25, -3.51)));
                path.add(new Line(new Point(4.25, -3.51), new Point(3.25, -3.5)));
                follow(path);
                turn(Functions.normalize(-robot.getAngle() + 180));
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(3000);
                path.add(new Line(odometry.getPoint(), new Point(3.5, -3.5)));
                backFollow(path);
                robot.setDrivePower(0, 0, 0, 0);
                strafeTurn(0, angleOffset, new Point(3.5, 3));
                strafeTurn(0, angleOffset, new Point(3, 3));
                robot.setDrivePower(0, 0, 0, 0);
                sleep(3000);
                path.clear();
                if(System.currentTimeMillis() - time <= 18000 || test) {
                    strafeTurn(0, angleOffset, new Point(3.3, 2.9));
                    robot.setDrivePower(0, 0, 0, 0);
                    path.clear();
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    strafeTurn(0, angleOffset, new Point(2.87, -4.98));
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(Functions.normalize(-robot.getAngle() + 25.7142857143 + 180));
                    sleep(3000);
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    path.add(new Line(odometry.getPoint(), new Point(3, odometry.getPoint().getY() + 0.01)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    strafeTurn(0, angleOffset, new Point(3.3, 3));
                    strafeTurn(0, angleOffset, new Point(2.6, 3));
                    path.clear();
                    robot.setDrivePower(0, 0, 0, 0);
                    sleep(1000);
                    robot.lockFoundation();
                    sleep(500);
                    strafeTurn(0, angleOffset, new Point(4, 4));
                    strafeTurn(0, angleOffset, new Point(4.6, 4));
                    path.add(new Line(odometry.getPoint(), new Point(4.5, 0)));
                }
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(4.5, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                }
            }
            if(skystone == 2) {
                path.add(new Line(odometry.getPoint(), new Point(4.25, -3.01)));
                path.add(new Line(new Point(4.25, -3.01), new Point(3.25, -3)));
                follow(path);
                turn(Functions.normalize(-robot.getAngle() + 180));
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(3000);
                path.add(new Line(odometry.getPoint(), new Point(3.5, -3)));
                backFollow(path);
                robot.setDrivePower(0, 0, 0, 0);
                strafeTurn(0, angleOffset, new Point(3.5, 3));
                strafeTurn(0, angleOffset, new Point(3, 3));
                robot.setDrivePower(0, 0, 0, 0);
                sleep(3000);
                path.clear();
                if(System.currentTimeMillis() - time <= 18000 || test) {
                    strafeTurn(0, angleOffset, new Point(3.3, 2.9));
                    robot.setDrivePower(0, 0, 0, 0);
                    path.clear();
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    strafeTurn(0, angleOffset, new Point(2.95, -4.98));
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    sleep(3000);
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    path.add(new Line(odometry.getPoint(), new Point(3, odometry.getPoint().getY() + 0.01)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    strafeTurn(0, angleOffset, new Point(3.3, 3));
                    strafeTurn(0, angleOffset, new Point(2.6, 3));
                    path.clear();
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    robot.lockFoundation();
                    sleep(500);
                    strafeTurn(0, angleOffset, new Point(4, 4));
                    strafeTurn(0, angleOffset, new Point(4.6, 4));
                    robot.unlockFoundation();
                    path.add(new Line(odometry.getPoint(), new Point(4.5, 0)));
                }
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(4.5, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                }
            }
            if(skystone == 3) {
                turnToPoint(new Point(4.45, -2.6));
                path.add(new Line(odometry.getPoint(), new Point(4.45, -2.6)));
                path.add(new Line(new Point(4.25, -2.6), new Point(3.25, -2.4)));
                follow(path);
                turn(Functions.normalize(-robot.getAngle() + 180));
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(3000);
                path.add(new Line(odometry.getPoint(), new Point(3.5, -2.6)));
                backFollow(path);
                robot.setDrivePower(0, 0, 0, 0);
                strafeTurn(0, angleOffset, new Point(3.5, 3));
                strafeTurn(0, angleOffset, new Point(3, 3));
                robot.setDrivePower(0, 0, 0, 0);
                sleep(3000);
                path.clear();
                if(System.currentTimeMillis() - time <= 18000 || test) {
                    strafeTurn(0, angleOffset, new Point(3.4, 2.9));
                    robot.setDrivePower(0, 0, 0, 0);
                    path.clear();
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    strafeTurn(0, angleOffset, new Point(3, -4.25));
                    robot.setDrivePower(0, 0, 0, 0);
                    path.add(new Line(odometry.getPoint(), new Point(3.25, -4.251)));
                    follow(path);
                    sleep(3000);
                    path.add(new Line(odometry.getPoint(), new Point(3, odometry.getPoint().getY() + 0.01)));
                    backFollow(path);
                    robot.setDrivePower(0, 0, 0, 0);
                    strafeTurn(0, angleOffset, new Point(3.3, 3));
                    strafeTurn(0, angleOffset, new Point(2.6, 3));
                    path.clear();
                    robot.setDrivePower(0, 0, 0, 0);
                    turn(Functions.normalize(-robot.getAngle() + 180));
                    robot.lockFoundation();
                    sleep(3000);
                    strafeTurn(Functions.normalize(0), angleOffset, new Point(4, 4));
                    strafeTurn(Functions.normalize(0), angleOffset, new Point(4.6, 4));
                    robot.unlockFoundation();
                    path.add(new Line(odometry.getPoint(), new Point(4.5, 0)));
                }
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(4.5, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                }
            }
        }
        else if(pathNum % pathNames.size() == 1) {
            radius = 0.75;
            turnCoeff = 0.003;
            if(skystone == 1) {
                turnToPoint(new Point(4.25, -3.51));
                path.add(new Line(odometry.getPoint(), new Point(4.25, -3.51)));
                path.add(new Line(new Point(4.25, -3.51), new Point(3.25, -3.5)));
                follow(path);
                if(Math.abs(Functions.normalize(-robot.getAngle() + Globals.START_THETA)) > 10) {
                    turn(-robot.getAngle() + Globals.START_THETA);
                }
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(3.5, -3.5));
                robot.setDrivePower(0, 0, 0, 0);
                recipBackFollow(3.97541861477, -15.7858849854, -1.05011960422, 4.25310999583, -3.5, 3, 4);
                strafeTurn(Functions.normalize(-robot.getAngle() + Globals.START_THETA), angleOffset, new Point(3, 4));
                robot.lockFoundation();
                sleep(500);
                strafeTurn(0, angleOffset, new Point(5, 4.15));
                robot.unlockFoundation();
                sleep(500);
                strafeTurn(0, angleOffset, new Point(4.8, 2.2));
                strafeTurn(Functions.normalize(-robot.getAngle() - 90 + Globals.START_THETA), angleOffset, new Point(4, 2.2));
                recipFollow(7.08837181224, -28.1610097592, -1.27843505112, 5.20122997255, 2.2, 2.79, 4);
                recipBackFollow(7.08837181224, -28.1610097592, -1.27843505112, 5.20122997255, -5.13, 2.79, 4);
                sleep(3000);
                recipFollow(2.56858790564, -10.1863516226, -1.03851648071, 4.19406592285, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(2.56858790564, -10.1863516226, -1.03851648071,4.19406592285, -2.3, 3, 4);
                sleep(3000);
                recipFollow(3.46410058868, -13.7294792778, -1.05470019623, 4.27649309259, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(3.46410058868, -13.7294792778, -1.05470019623, 4.27649309259, -3, 3, 4);
                sleep(3000);
                recipFollow(4.74026383952, -18.8587456386, -1.04452219231, 4.22459318717, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(4.74026383952, -18.8587456386, -1.04452219231, 4.22459318717, -4.251, 3, 4);
                sleep(3000);
                recipFollow(5.48004245547, -21.8282478163, -1.04016916777, 4.20245940087, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(5.48004245547, -21.8282478163, -1.04016916777, 4.20245940087, -4.98, 3, 4);
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                    double displacement = Math.abs(odometry.getPoint().distance(new Point(odometry.getX(), 0)) * -Globals.DRIVE_FEET_PER_TICK);
                    if(Math.abs(displacement) <= (Math.sqrt(2) / 10)) {
                        telemetry.addData("Time elapsed", (System.currentTimeMillis() - starttime) / 1000.0);
                        telemetry.update();
                        while(opModeIsActive()) {
                            wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
                            odometry.update();
                        }
                    }
                }
            }
            if(skystone == 2) {
                path.add(new Line(odometry.getPoint(), new Point(4.25, -3.01)));
                path.add(new Line(new Point(4.25, -3.01), new Point(3.25, -3)));
                follow(path);
                if(Math.abs(Functions.normalize(-robot.getAngle() + Globals.START_THETA)) > 10) {
                    turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
                }
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(3.5, -3));
                robot.setDrivePower(0, 0, 0, 0);
                recipBackFollow(3.46410058868, -13.7294792778, -1.05470019623, 4.27649309259, -3, 3, 4);
                /*
                turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
                strafeTurn(0, angleOffset, new Point(-2.6, 3));
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(-5, 3));
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(odometry.getX(), 2.2));
                strafeTurn(Functions.normalize(-robot.getAngle() - 90), angleOffset, new Point(-4, 2.2));
                 */
                strafeTurn(Functions.normalize(-robot.getAngle() + Globals.START_THETA), angleOffset, new Point(2.65, 4));
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(5, 4.15));
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(4.8, 2.2));
                strafeTurn(Functions.normalize(-robot.getAngle() - 90 + Globals.START_THETA), angleOffset, new Point(4, 2.2));
                recipFollow(5.48004245547, -21.8282478163, -1.04016916777, 4.20245940087, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(5.48004245547, -21.8282478163, -1.04016916777, 4.20245940087, -4.98, 3, 4);
                sleep(3000);
                recipFollow(2.56858790564, -10.1863516226, -1.03851648071, 4.19406592285, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(2.56858790564, -10.1863516226, -1.03851648071,4.19406592285, -2.3, 3, 4);
                sleep(3000);
                recipFollow(3.97541861477, -15.7858849854, -1.05011960422, 4.25310999583, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(3.97541861477, -15.7858849854, -1.05011960422, 4.25310999583, -3.5, 3, 4);
                sleep(3000);
                recipFollow(4.74026383952, -18.8587456386, -1.04452219231, 4.22459318717, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(4.74026383952, -18.8587456386, -1.04452219231, 4.22459318717, -4.251, 3, 4);
                sleep(3000);
                recipFollow(7.08837181224, -28.1610097592, -1.27843505112, 5.20122997255, 2.2, 2.79, 4);
                sleep(3000);
                recipBackFollow(7.08837181224, -28.1610097592, -1.27843505112, 5.20122997255, -5.13, 2.79, 4);
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                    double displacement = Math.abs(odometry.getPoint().distance(new Point(odometry.getX(), 0)) * -Globals.DRIVE_FEET_PER_TICK);
                    if(Math.abs(displacement) <= (Math.sqrt(2) / 10)) {
                        telemetry.addData("Time elapsed", (System.currentTimeMillis() - starttime) / 1000.0);
                        telemetry.update();
                        while(opModeIsActive()) {
                            wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
                            odometry.update();
                        }
                    }
                }
            }
            if(skystone == 3) {
                turnToPoint(new Point(4.25, -2.6));
                path.add(new Line(odometry.getPoint(), new Point(4.45, -2.6)));
                path.add(new Line(new Point(4.25, -2.6), new Point(3.25, -2.4)));
                follow(path);
                if(Math.abs(Functions.normalize(robot.getAngle() + Globals.START_THETA)) > 10) {
                    turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
                }
                robot.setDrivePower(0, 0, 0, 0);
                path.clear();
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(3.5, -2.4));
                robot.setDrivePower(0, 0, 0, 0);
                recipBackFollow(2.56858790564, -10.1863516226, -1.03851648071,4.19406592285, -2.3, 3, 4);
                strafeTurn(Functions.normalize(-robot.getAngle()), angleOffset, new Point(2.65, 4));
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(5, 4.15));
                sleep(3000);
                strafeTurn(0, angleOffset, new Point(4.8, 2.2));
                strafeTurn(Functions.normalize(-robot.getAngle() - 90 + Globals.START_THETA), angleOffset, new Point(4, 2.2));
                recipBackFollow(4.74026383952, -18.8587456386, -1.04452219231, 4.22459318717, -4.251, 3, 4);
                sleep(3000);
                recipFollow(3.46410058868, -13.7294792778, -1.05470019623, 4.27649309259, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(3.46410058868, -13.7294792778, -1.05470019623, 4.27649309259, -3, 3, 4);
                sleep(3000);
                recipFollow(3.97541861477, -15.7858849854, -1.05011960422, 4.25310999583, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(3.97541861477, -15.7858849854, -1.05011960422, 4.25310999583, -3.5, 3, 4);
                sleep(3000);
                recipFollow(5.48004245547, -21.8282478163, -1.04016916777, 4.20245940087, 2.2, 3, 4);
                sleep(3000);
                recipBackFollow(5.48004245547, -21.8282478163, -1.04016916777, 4.20245940087, -4.98, 3, 4);
                sleep(3000);
                recipFollow(7.08837181224, -28.1610097592, -1.27843505112, 5.20122997255, 2.2, 2.79, 4);
                sleep(3000);
                recipBackFollow(7.08837181224, -28.1610097592, -1.27843505112, 5.20122997255, -5.13, 2.79, 4);
                while (opModeIsActive()) {
                    robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
                    odometry.update();
                    double displacement = Math.abs(odometry.getPoint().distance(new Point(odometry.getX(), 0)) * -Globals.DRIVE_FEET_PER_TICK);
                    if (Math.abs(displacement) <= (Math.sqrt(2) / 10)) {
                        telemetry.addData("Time elapsed", (System.currentTimeMillis() - starttime) / 1000.0);
                        telemetry.update();
                        while (opModeIsActive()) {
                            wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
                            odometry.update();
                        }
                    }
                }
            }
        }
        else if(pathNum % pathNames.size() == 2) {
            radius = 0.5;
            turnCoeff = 0.0055 ;
            strafeTurn(0, angleOffset, new Point(2.35, 3));
            sleep(3000);
            strafeTurn(-odometry.getAngle(), angleOffset, new Point(4, 4));
            strafeTurn(-odometry.getAngle(), angleOffset, new Point(4.6, 4));
            path.add(new Line(odometry.getPoint(), new Point(4.5, 0)));
            while (opModeIsActive()) {
                robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                wheels.update(robot, new Point(4.4, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                odometry.update();
                double displacement = Math.abs(odometry.getPoint().distance(new Point(4.4, 0)) * -Globals.DRIVE_FEET_PER_TICK);
                if (Math.abs(displacement) <= (Math.sqrt(2) / 10)) {
                    while (opModeIsActive()) {
                        wheels.update(robot, new Point(4.4, 0), odometry, Double.NaN, AngleUnit.DEGREES);
                        odometry.update();
                    }
                }
            }
        }
        else if(pathNum % pathNames.size() == 3) {
            double time = System.currentTimeMillis();
            if(skystone == 0) {
                strafeTurn(0, angleOffset, new Point(3.3, -2.3));
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -2.3));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3, 4));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -4.25));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.3, -4.25));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -4.25));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                if(System.currentTimeMillis() - time >= 20000) {
                    strafeTurn(0, angleOffset, new Point(2.75, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    robot.unlockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(5.1, 4));
                    odometry.update();
                    robot.lockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(4.8, 0));
                    odometry.update();
                }
                else {
                    strafeTurn(0, angleOffset, new Point(3, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    sleep(2000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, 4));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, -3));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.3, -3));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    sleep(2000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, -3));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, 4));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(2.75, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    robot.unlockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(5.1, 4));
                    odometry.update();
                    robot.lockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(4.8, 0));
                    odometry.update();
                }
            }
            else if (skystone == 1) {
                strafeTurn(0, angleOffset, new Point(3.3, -3));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -3));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3, 4));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -4.95));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.3, -4.95));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -4.95));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                if(System.currentTimeMillis() - time >= 20000) {
                    strafeTurn(0, angleOffset, new Point(2.75, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    robot.unlockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(5.1, 4));
                    odometry.update();
                    robot.lockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(4.8, 0));
                    odometry.update();
                }
                else {
                    strafeTurn(0, angleOffset, new Point(3, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    sleep(2000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, 4));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, -2.3));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.3, -2.3));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    sleep(2000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, -2.3));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, 4));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(2.75, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    robot.unlockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(5.1, 4));
                    odometry.update();
                    robot.lockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(4.8, 0));
                    odometry.update();
                }
            }
            else {
                strafeTurn(0, angleOffset, new Point(3.3, -3.6));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -3.6));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3, 4));
                odometry.update();
                turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, -4.95));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3, -4.95));
                odometry.update();
                turn(Functions.normalize(30 + -(robot.getAngle() - Globals.START_THETA)));
                odometry.update();
                sleep(2000);
                odometry.update();
                turn(-30);
                strafeTurn(0, angleOffset, new Point(3.7, -4.95));
                odometry.update();
                strafeTurn(0, angleOffset, new Point(3.7, 4));
                odometry.update();
                if(System.currentTimeMillis() - time >= 20000) {
                    strafeTurn(0, angleOffset, new Point(2.75, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    robot.unlockFoundation();
                    sleep(1000);
                    strafeTurn(0, angleOffset, new Point(5.1, 4));
                    odometry.update();
                    robot.lockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(4.8, 0));
                    odometry.update();
                }
                else {
                    strafeTurn(0, angleOffset, new Point(3, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    sleep(2000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, 4));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, -2.3));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.3, -2.3));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    sleep(2000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, -2.3));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(3.7, 4));
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(2.75, 4));
                    odometry.update();
                    turn(Functions.normalize(-(robot.getAngle() - Globals.START_THETA)));
                    odometry.update();
                    robot.unlockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(5.1, 4));
                    odometry.update();
                    robot.lockFoundation();
                    sleep(1000);
                    odometry.update();
                    strafeTurn(0, angleOffset, new Point(4.8, 0));
                    odometry.update();
                }
            }
        }
        while (opModeIsActive()) {
            robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
            odometry.update();
            double displacement = Math.abs(odometry.getPoint().distance(new Point(odometry.getX(), 0)) * -Globals.DRIVE_FEET_PER_TICK);
            if (Math.abs(displacement) <= (Math.sqrt(2) / 10)) {
                telemetry.addData("Time elapsed", (System.currentTimeMillis() - starttime) / 1000.0);
                telemetry.update();
                while (opModeIsActive()) {
                    wheels.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, AngleUnit.DEGREES);
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
                    if(fastturns && relAngle > 13.516 && odometry.getY() < -1.2) { // It looks calculated, but I really just wanted to throw the number "516" somewhere random
                        turnCoeff = 0.1 ;
                    }
                    else if(fastturns) {
                        turnCoeff = 0.006 ;
                    }
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
                                angle = Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                                //drive += 0.5 * PIDd;
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
                line = new Line(odometry.getPoint(), line.getPoint2());
                nextPoint = Functions.lineCircleIntersection(new Circle(odometry.getPoint(), radius), line);
                thing = true;
                if(k == path.size() - 1 && nextPoint.size() == 0 && !done) {
                    strafeTurn(0, angleOffset, line.getPoint2());
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
        boolean done = false;
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
                    if(fastturns && relAngle > 13.516) { // It looks calculated, but I really just wanted to throw the number "516" somewhere random
                        turnCoeff = 0.045 ;
                    }
                    else if(fastturns) {
                        turnCoeff = 0.003 ;
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
                                angle = Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                                //drive += 0.5 * PIDd;
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
                if(k == path.size() - 1 && nextPoint.size() == 0 && !done) {
                    strafeTurn(0, angleOffset, line.getPoint2());
                    done = true;
                }
            }
        }
        robot.setDrivePower(0, 0, 0, 0);
    }

    /**
     * Generate Path for equation in the form y = ax + b / cx + d
     * @param a A coefficient
     * @param b B term
     * @param c C coefficient
     * @param d D term
     * @param bound2 Upper y bound
     * @param x1 Start x position
     * @param x2 End x position
     * @throws InterruptedException If OpMode is interrupted, throw an exception
     */
    public void recipFollow(double a, double b, double c, double d, double bound2, double x1, double x2) throws InterruptedException {
        /*
        if(System.currentTimeMillis() - starttime >= 28500) {
            strafeTurn(0, angleOffset, new Point(odometry.getX(), 0));
            while(opModeIsActive()) {}
            throw new InterruptedException("Interrupted");
        }
         */
        ArrayList<Line> path = new ArrayList<>();
        path.add(new Line(odometry.getPoint(), new Point(x2 - 0.01, ((a * (x2 - 0.01)) + b) / ((c * (x2 - 0.01)) + d))));
        for (double x = x2 - 0.02; x > x1; x -= 0.01) {
            path.add(new Line(path.get(path.size() - 1).getPoint2(), new Point(x, ((a * x) + b) / ((c * x) + d))));
        }
        fastturns = true;
        follow(path);
        fastturns = false;
        if(Math.abs(a) != 7.08837181224) {
            strafeTurn(Functions.normalize(-robot.getAngle() + Globals.START_THETA), angleOffset, new Point(odometry.getX() - 0.24, odometry.getY()));
        }
        else {
            strafeTurn(0, angleOffset, new Point(odometry.getX() - 0.23, odometry.getY()));
        }
        robot.setDrivePower(0, 0, 0, 0);
    }

    /**
     * Generate backwards Path for equation in the form y = ax + b / cx + d
     * @param a A coefficient
     * @param b B term
     * @param c C coefficient
     * @param d D term
     * @param bound1 Lower y bound
     * @param x1 Start x position
     * @param x2 End x position
     * @throws InterruptedException If OpMode is interrupted, throw an exception
     */
    public void recipBackFollow(double a, double b, double c, double d, double bound1, double x1, double x2)  throws InterruptedException {
        /*
        if(System.currentTimeMillis() - starttime >= 28500) {
            strafeTurn(0, angleOffset, new Point(odometry.getX(), 0));
            while(opModeIsActive()) {}
            throw new InterruptedException("Interrupted");
        }
         */
        ArrayList<Line> path = new ArrayList<>();
        path.add(new Line(odometry.getPoint(), new Point(x1, bound1)));
        path.add(new Line(new Point(x1, bound1), new Point(x1 + 0.01, ((a * (x1 + 0.01)) + b) / ((c * (x1 + 0.01)) + d))));
        for(double x = x1 + 0.02; x < x2; x += 0.01) {
            path.add(new Line(path.get(path.size() - 1).getPoint2(), new Point(x, ((a * x) + b) / ((c * x) + d))));
        }
        fastturns = true;
        backFollow(path);
        fastturns = false;
        turn(Functions.normalize((90.0 * Math.round(robot.getAngle() / 90.0)) - robot.getAngle() + Globals.START_THETA));
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
     * Turn a certain, specified amount
     * @param angle Amount to turn
     */
    public void turn(double angle) {
        if(Math.abs(angle) < angleOffset || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double angleIntended;
        double robotAngle;
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = odometry.getAngle();
        angleIntended = Functions.normalize(robotAngle + angle);
        boolean first = true;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();
        do {
            double error = Functions.normalize(angleIntended - odometry.getAngle());
            while (opModeIsActive() && Math.abs(error) > angleOffset) {
                odometry.update();
                error = Functions.normalize(angleIntended - odometry.getAngle());
                if (Math.abs(error + 360) < Math.abs(error)) {
                    error += 360;
                }
                if (Math.abs(error - 360) < Math.abs(error)) {
                    error -= 360;
                }
                double delta = lastError - error;
                double deltaTime = System.currentTimeMillis() - lastTime;
                double deriv = 0;
                if(!first) {
                    deriv = delta / deltaTime;
                }
                double Kp = 0.02 / 2f;
                double Kd = -0.0125 / 2f;
                double pow = (Kp * error) + (Kd * deriv);
                pow = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                robot.setDrivePower(pow, pow, -pow, -pow);
                first = false;
                lastError = error;
                lastTime = System.currentTimeMillis();
            }
            robot.setDrivePower(0, 0, 0, 0);
            odometry.update();
        }
        while(Math.abs(odometry.getAngle() - angleIntended) > angleOffset);
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
     * Simultaneously strafe to a certain point and turn to a certain angle
     * @param angle Amount to turn during movement
     * @param angleOffset Turning tolerance
     * @param target Point to which to strafe
     */
    private void strafeTurn(double angle, double angleOffset, Point target) {
        if(angle == 0) {
            odometry.update();
            double displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            double startdp = displacement;
            double timer = System.currentTimeMillis();
            while(Math.abs(displacement) > 0.15) {
                wheels.update(robot, target, odometry, Double.NaN, AngleUnit.DEGREES);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                if(System.currentTimeMillis() - timer >= ((startdp * 3000) + 2000) && Math.abs(odometry.getVelocity()) <= 0.08) {
                    break;
                }
            }
            robot.setDrivePower(0, 0, 0, 0);
            return;
        }
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
                if(Math.max(Math.abs(strafePows[0] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs(strafePows[1] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs((strafePows[2] + Math.max(0.03 * error, 0.1))), Math.abs((strafePows[3] + Math.max(0.03 * error, 0.1)))))) > 1) {
                    scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(strafePows[0] + Math.min(-0.03 * error, -0.1)), Math.max(Math.abs(strafePows[1] + Math.min(-0.02 * error, -0.1)), Math.max(Math.abs((strafePows[2] + Math.max(0.02 * error, 0.1))), Math.abs((strafePows[3] + Math.max(0.03 * error, 0.1))))));
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
        double startdp = displacement;
        double timer = System.currentTimeMillis();
        while(Math.abs(displacement) > 0.15) {
            wheels.update(robot, target, odometry, angle, AngleUnit.DEGREES);
            displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            if(System.currentTimeMillis() - timer >= ((startdp * 1000) + 1000) && Math.abs(odometry.getVelocity()) <= 0.08) {
                break;
            }
        }
        robot.setDrivePower(0, 0, 0, 0);
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
