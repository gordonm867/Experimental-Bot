package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

import java.util.ArrayList;

public class Drivetrain {

    private State state;

    public enum State {
        DRIVING,
        STOPPED
    }

    public Drivetrain(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot) {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double angle = gamepad1.left_stick_x;
        if(state == State.DRIVING) {
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
            } else {
                scaleFactor = 1;
            }
            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
            robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
        }
        else if(state == State.STOPPED) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(double lr, double lf, double rr, double rf, TrashHardware robot) {
        if(state == State.DRIVING) {
            robot.setDrivePower(lr, lf, rr, rf); // Set motors to values based on gamepad
        }
        else if(state == State.STOPPED) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(ArrayList<Double> powers, TrashHardware robot) {
        if(state == State.DRIVING) {
            robot.setDrivePower(powers.get(0), powers.get(1), powers.get(2), powers.get(3)); // Set motors to values based on gamepad
        }
        else if(state == State.STOPPED) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(double[] powers, TrashHardware robot) {
        if(state == State.DRIVING) {
            robot.setDrivePower(powers[0], powers[1], powers[2], powers[3]); // Set motors to values based on gamepad
        }
        else if(state == State.STOPPED) {
            robot.setDrivePower(0, 0, 0, 0);
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
