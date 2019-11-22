package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.BoxLift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Autonomous(name="Insanity Autonomous Test", group="Trash")
@Config
//@Disabled
public class AutoTest extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Extension extension;
    private BoxLift boxLift;
    private Intake intake;
    private FoundationMover mover;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);

    Point target;

    public static double TARGET_THETA = Globals.START_THETA;
    public static int READY = 0;
    public static int index = 0;

    private double time = 0;
    private double opTime = 0;

    ArrayList<Point> targets = new ArrayList<>();
    ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void runOpMode() throws InterruptedException {

        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);

        targets.add(new Point(-5.25, -3, 180));
        targets.add(new Point(-2.25, -4.4, -90));
        targets.add(new Point(-2.25, -3.8, -90));
        targets.add(new Point(-2.9, -3.8, -90));
        targets.add(new Point(-2.9, 3, 180));
        targets.add(new Point(-2.9, -3.3, 90));
        targets.add(new Point(-2.25, -3.3, 90));
        targets.add(new Point(-2.25, -4, 90));
        targets.add(new Point(-2.9, -4, 90));
        targets.add(new Point(-2.9, 3, 180));
        targets.add(new Point(-2.9, 0, 90));

        boxLift = new BoxLift(BoxLift.State.NEUTRAL);
        extension = new Extension(Extension.State.IDLE);
        mover = new FoundationMover(FoundationMover.State.UNLOCKED);
        intake = new Intake(Intake.State.IDLE);
        odometry = Odometry.getInstance(robot);

        subsystems.add(boxLift);
        subsystems.add(extension);
        subsystems.add(mover);
        subsystems.add(intake);
        robot.enabled = true;

        if(odometry.getPoint().equals(new Point(-4, -2))) {
            Globals.MAX_SPEED = 0.4;
            READY = 1;
            target = targets.get(0);
            double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
            while(((displacement >= 0.15 || (Functions.normalize(Math.abs(odometry.getAngle() - TARGET_THETA))) >= 1) && READY != 0)) {
                RevBulkData data = robot.bulkRead();
                odometry.update(data);
                displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
                TARGET_THETA = target.getAngle();
                if(Double.isNaN(TARGET_THETA)) {
                    TARGET_THETA = odometry.getAngle();
                }
                if((displacement >= 0.15 || (Functions.normalize(Math.abs(odometry.getAngle() - TARGET_THETA))) >= 1) && READY != 0) {
                    wheels.update(robot, target, odometry, TARGET_THETA, AngleUnit.DEGREES);
                }
            }
            index++;
            READY = 0;
        }
        Globals.MAX_SPEED = 1.0;
        while(!isStarted() && !isStopRequested()) {
            robot.setDrivePower(0, 0, 0, 0);
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        waitForStart();
        opTime = System.currentTimeMillis();
        wheels.setState(Drivetrain.State.ON);
        READY = 0;
        target = targets.get(index);
        while(opModeIsActive()) {
            RevBulkData data = robot.bulkRead();
            RevBulkData data2 = robot.bulkReadTwo();
            odometry.update(data);
            double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
            target = targets.get(index);
            TARGET_THETA = target.getAngle();
            if(Double.isNaN(TARGET_THETA)) {
                TARGET_THETA = odometry.getAngle();
            }
            if((displacement >= 0.15 || (Functions.normalize(Math.abs(odometry.getAngle() - TARGET_THETA))) >= 1) && READY != 0) {
                wheels.update(robot, target, odometry, TARGET_THETA, AngleUnit.DEGREES);
            }
            else {
                robot.setDrivePower(0, 0, 0, 0);
                for(Subsystem subsystem : subsystems) {
                    subsystem.update(gamepad1, gamepad2, robot, data, data2);
                }
                if(READY != 0) {
                    time = System.currentTimeMillis();
                    READY = 0;
                    index++;
                    if(index == targets.size()) {
                        double elapsed = (System.currentTimeMillis() - opTime) / 1000.0;
                        while(opModeIsActive()) {
                            telemetry.addData("Time elapsed", elapsed);
                            telemetry.update();
                        }
                    }
                }

                /*
                if(System.currentTimeMillis() - time >= 500 && READY == 0) {
                    READY = 1;
                }

                 */
            }
            telemetry.addData("Point", odometry.getPoint());
            telemetry.addData("Angle", odometry.getAngle());
            telemetry.addData("Target Point", target);
            telemetry.addData("Target Angle", TARGET_THETA);
            telemetry.addData("Displacement", displacement);
            telemetry.addData("Angular Displacement", Math.abs(odometry.getAngle() - TARGET_THETA));
            telemetry.update();
        }
    }
}
