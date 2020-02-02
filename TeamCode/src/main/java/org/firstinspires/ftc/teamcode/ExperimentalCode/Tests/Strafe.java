package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;

@TeleOp(name="TrashTest",group="Tests")
@Disabled
public class Strafe extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);
    private Gamepad rest = new Gamepad();
    private boolean automation = false;
    private boolean isTransitioning = false;
    private int trans = 0;

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        odometry = Odometry.getInstance(robot);
        robot.enabled = true;
        while(!gamepad1.atRest() || !gamepad2.atRest()) {}
        try {
            rest.copy(gamepad1);
        }
        catch(Exception p_exception) {
            telemetry.addData("Warning", "Expect serious issues during this OpMode.");
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        wheels.setState(Drivetrain.State.ON);
        while(opModeIsActive()) {
            Gamepad gamepad3 = new Gamepad();

            if(!isTransitioning && ((gamepad1.start && gamepad1.x) || (gamepad2.start && gamepad2.x))) {
                if(gamepad2.start && gamepad2.x) {
                    trans = 2;
                }
                else {
                    trans = 1;
                }
                isTransitioning = true;
                automation = !automation;
            }

            if(!((gamepad1.start && gamepad1.x) || (gamepad2.start && gamepad2.x))) {
                isTransitioning = false;
            }

            if(automation) {
                try {
                    gamepad3.copy(trans == 1 ? gamepad1 : gamepad2);
                    (trans == 1 ? gamepad1 : gamepad2).copy(rest);
                }
                catch(Exception p_exception) {
                    automation = false;
                }
            }
            else {
                try {
                    gamepad3.copy(rest);
                } catch (Exception p_exception) {
                    automation = false;
                }
            }

            while(opModeIsActive() && automation) {
                Point myPos = odometry.getPoint();
                Point target = new Point(-4, 0);
                double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
                double angle = 0;
                double drive = 0;
                double turn = 0;
                if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
                    double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                    if(PIDd != -displacement) {
                        angle = Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(odometry.getAngle())) * displacement;
                        drive = PIDd;
                        if(Math.abs(displacement) <= (Math.sqrt(2) / 10) || (Math.abs(angle) < 0.001 && Math.abs(drive) < 0.001)) {
                            drive = 0;
                            angle = 0;
                        }
                    }
                }
                double scaleFactor;
                if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
                    scaleFactor = Math.abs(gamepad3.left_stick_y) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
                } else {
                    scaleFactor = Math.abs(gamepad3.left_stick_y) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
                }
                robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
                odometry.update();
            }

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double angle = gamepad1.left_stick_x;

            /* Precision vertical drive */
            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                if (gamepad1.left_stick_y != 0) {
                    drive = drive * 0.2; // Slow down joystick driving
                } else {
                    if (gamepad1.dpad_down) {
                        drive = -0.2; // Slow drive backwards
                    } else {
                        drive = 0.2; // Slow drive forwards
                    }
                }
            }

            /* Precision sideways drive */
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                if (gamepad1.right_stick_x != 0) {
                    angle = angle * 0.3; // Slow down joystick side movement
                } else {
                    if (gamepad1.dpad_left) {
                        angle = -0.3; // Slow leftwards
                    } else {
                        angle = 0.3; // Slow rightwards
                    }
                }
            }

            /* Precision turn */
            if (gamepad1.left_bumper) {
                turn = -0.2; // Slow left turn
            }
            if (gamepad1.right_bumper) {
                turn = 0.2; // Slow right turn
            }

            drive = adjust(drive);
            turn = adjust(turn);
            angle = adjust(angle);
            double scaleFactor;

            if (Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))) > 1) {
                scaleFactor = Globals.MAX_SPEED / (Math.max(Math.abs(drive + turn + angle), Math.abs(drive - turn - angle)));
            }
            else {
                scaleFactor = 1;
            }

            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
            robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
            odometry.update();
        }
    }

    private double adjust(double varToAdjust) { // Square-root driving
        if (varToAdjust < 0) {
            varToAdjust = -Math.sqrt(-varToAdjust);
        } else {
            varToAdjust = Math.sqrt(varToAdjust);
        }
        return varToAdjust;
    }

}
