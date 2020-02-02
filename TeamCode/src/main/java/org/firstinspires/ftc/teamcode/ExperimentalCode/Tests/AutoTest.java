package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Autonomous(name="Insanity Autonomous Test", group="Trash")
//@Config
@Disabled
public class AutoTest extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    //private Extension extension;
    //private BoxLift boxLift;
    //private Intake intake;
    //private FoundationMover mover;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);

    Point target;

    public static double TARGET_THETA = Globals.START_THETA;
    public static int READY = 0;
    public static int index = 0;

    private double time = 0;
    private double opTime = 0;
    private double wait = 0;

    private boolean offed = false;

    ArrayList<Point> targets = new ArrayList<>();
    ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void runOpMode() throws InterruptedException {

        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.unlockFoundation();
        Globals.boxDown = 0.395;

        targets.add(new Point(-5.25, -3.5, 180));
        targets.add(new Point(-2.6, -3.7, 135));
        targets.add(new Point(-3.4, -3.5, 150));
        targets.add(new Point(-3.4, 4, 180));
        targets.add(new Point(-2.5, 4, 180));
        targets.add(new Point(-3.5, -1.5, 135));
        targets.add(new Point(-2.8, -1.65, 135));
        targets.add(new Point(-3.4, -0.5, 150));
        targets.add(new Point(-3.4, 4, 180));
        targets.add(new Point(-2.4, 4, 180));
        targets.add(new Point(-4.5, 4.2, -135));
        targets.add(new Point(-3.25, 0, -135));

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
        READY = 1;
        target = targets.get(index);
        while(opModeIsActive()) {
            RevBulkData data = robot.bulkRead();
            //RevBulkData data2 = robot.bulkReadTwo();
            odometry.update(data);
            double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - odometry.getX(), 2) + Math.pow(target.getY() - odometry.getY(), 2)));
            target = targets.get(index);
            TARGET_THETA = target.getAngle();
            if(Double.isNaN(TARGET_THETA)) {
                TARGET_THETA = odometry.getAngle();
            }
            if(index == 5 && !offed) {
                robot.setInPowerTest(0);
                offed = true;
            }
            if((displacement >= 0.15 || (Functions.normalize(Math.abs(odometry.getAngle() - TARGET_THETA))) >= 1.5) && READY != 0) {
                wheels.update(robot, target, odometry, TARGET_THETA, AngleUnit.DEGREES);
                if((index == 1 && Math.abs(odometry.getX()) <= 5.3) || (index == 5 && odometry.getY() <= 0)) {
                    if(wait == 0) {
                        wait = System.currentTimeMillis();
                    }
                    robot.liftbox(Globals.boxDown);
                    if(robot.ex != null && System.currentTimeMillis() - wait >= 200) {
                        robot.ex.setTargetPosition(Globals.EXTEND_POS);
                        robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.ex.setPower(0.75);
                        robot.setInPowerTest(-1);
                    }
                }
                if(((index == 1 && Math.abs(odometry.getX()) <= 3.75) || (index == 5 && odometry.getY() <= -2)) && robot.ex != null) {
                    if(robot.ex.isBusy()) {
                        Globals.MAX_SPEED = 0.5;
                    }
                    else {
                        Globals.MAX_SPEED = 1.0;
                    }
                }
                if((index == 2 || index == 7) && displacement <= 0.5) {
                    index++;
                }
            }
            else {
                if(READY != 0 && index != 2 && index != 3 && index != 5 && index != 7 && index != 8) {
                    robot.setDrivePower(0, 0, 0, 0);
                }
                odometry.update(data);
                /*
                for(Subsystem subsystem : subsystems) {
                    subsystem.update(gamepad1, gamepad2, robot, data, data2);
                }
                 */
                if(READY != 0) {
                    time = System.currentTimeMillis();
                    READY = 0;
                    index++;
                    if(index == targets.size()) {
                        double elapsed = (System.currentTimeMillis() - opTime) / 1000.0;
                        while (opModeIsActive()) {
                            telemetry.addData("Time elapsed", elapsed);
                            telemetry.update();
                        }
                    }
                }
                if((Math.abs(System.currentTimeMillis() - time) >= 200 || index == 3 || index == 4 || index == 6 || index == 8 || index == 9) && READY == 0) { // AFTER PAUSE, BEFORE MOVING ON
                    if(index == 2) {
                        Globals.MAX_SPEED = 1.0;
                        wait = 0;
                        robot.liftbox(Globals.boxNeutral);
                        if(robot.ex != null) {
                            robot.ex.setTargetPosition(0);
                            robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.ex.setPower(1);
                        }
                        robot.setInPowerTest(0);
                    }
                    if(index == 3 && robot.ex != null) {
                        while(Math.abs(robot.ex.getCurrentPosition()) >= 250) {
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                    }
                    if(index == targets.size() - 1) {
                        robot.unlockFoundation();
                        sleep(500);
                    }
                    READY = 1;
                }
                else if(READY == 0) { // DURING PAUSE
                    if(index == 2 || index == 7) {
                        if(index == 7) {
                            double tracktime = System.currentTimeMillis();
                            while (System.currentTimeMillis() - tracktime <= 2000) {
                                wheels.update(robot, new Point(-2.8, -1.65), odometry, 180, AngleUnit.DEGREES);
                            }
                        }
                        else {
                            sleep(2000);
                        }
                    }
                    if(index == 5 || index == 10) {
                        if (wait == 0) {
                            wait = System.currentTimeMillis();
                        }
                        robot.liftbox(0.41);
                        if (robot.ex != null && Math.abs(System.currentTimeMillis() - wait) >= 100) {
                            robot.ex.setTargetPosition(Globals.EXTEND_POS + 400);
                            robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.ex.setPower(0.85);
                            robot.setInPower(1);
                            double tracktime = System.currentTimeMillis();
                            while(System.currentTimeMillis() - tracktime <= 3500 && odometry.getX() <= (index == 10 ? -2.8 : -3.1)) {
                                wheels.update(robot, new Point((index == 10 ? -2.8 : -3.1), 3), odometry, 180, AngleUnit.DEGREES);
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            if(index == 10) {
                                Globals.MAX_SPEED = 0.2;
                                double foundtime = System.currentTimeMillis();
                                while(System.currentTimeMillis() - foundtime <= 1500) {
                                    wheels.update(robot, new Point(odometry.getX() + 0.5, 3), odometry, 180, AngleUnit.DEGREES);
                                }
                                robot.setDrivePower(0, 0, 0, 0);
                                robot.lockFoundation();
                                Globals.MAX_SPEED = 1.0;
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                    }
                }
            }
        }
    }
}
