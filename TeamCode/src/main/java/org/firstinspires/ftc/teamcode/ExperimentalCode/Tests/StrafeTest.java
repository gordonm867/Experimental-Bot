package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.BoxLift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name="Insanity Test", group="Trash")
@Config
@Disabled
public class StrafeTest extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Extension extension;
    private BoxLift boxLift;
    private Intake intake;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);

    Point target;

    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double TARGET_THETA = Globals.START_THETA;
    public static int READY = 0;

    ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void runOpMode() throws InterruptedException {

        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);

        boxLift = new BoxLift(BoxLift.State.NEUTRAL);
        extension = new Extension(Extension.State.IDLE);
        intake = new Intake(Intake.State.IDLE);
        odometry = Odometry.getInstance(robot);

        subsystems.add(boxLift);
        subsystems.add(extension);
        subsystems.add(intake);
        robot.enabled = true;

        robot.dropOdometry();

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        waitForStart();
        wheels.setState(Drivetrain.State.ON);
        TARGET_X = odometry.getX();
        TARGET_Y = odometry.getY();
        TARGET_THETA = odometry.getAngle();
        target = new Point(TARGET_X, TARGET_Y);
        while(opModeIsActive()) {
            RevBulkData data = robot.bulkRead();
            RevBulkData data2 = robot.bulkReadTwo();
            odometry.update(data);
            double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
            telemetry.addData("Point", odometry.getPoint());
            telemetry.addData("Angle", odometry.getAngle());
            telemetry.addData("Target Point", target);
            telemetry.addData("Target Angle", TARGET_THETA);
            telemetry.addData("Displacement", displacement);
            telemetry.addData("Angular Displacement", Math.abs(odometry.getAngle() - TARGET_THETA));
            telemetry.update();
            if(gamepad1.a && !gamepad1.start) {
                TARGET_X = 2;
                TARGET_Y = 2;
            }
            if(gamepad1.b) {
                TARGET_X = -2;
                TARGET_Y = 2;
            }
            if(gamepad1.x) {
                TARGET_X = 2;
                TARGET_Y = -2;
            }
            if(gamepad1.y) {
                TARGET_X = -2;
                TARGET_Y = -2;
            }
            if(gamepad1.dpad_left) {
                TARGET_X = 1.5;
                TARGET_Y = 0;
            }
            if(gamepad1.dpad_right) {
                TARGET_X = 1.56;
                TARGET_Y = 0.8;
            }
            target = new Point(TARGET_X, TARGET_Y);
            if((displacement >= 0.15 || (Math.abs(odometry.getAngle() - TARGET_THETA)) >= 1) && READY != 0) {
                wheels.update(robot, target, odometry, TARGET_THETA, AngleUnit.DEGREES);
            }
            else {
                robot.setDrivePower(0, 0, 0, 0);
            }
            for(Subsystem subsystem : subsystems) {
                subsystem.update(gamepad1, gamepad2, robot, data, data2);
            }
        }
    }
}
