package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;

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

    public double[] calcUpdate(Point target, Odometry odometry) {
        Point myPos = odometry.getPoint();
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
            scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        } else {
            scaleFactor = 0.2 / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        }
        double[] powers = {scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)};
        odometry.update();
        return powers;
    }

    public void update(TrashHardware robot, Point target, Odometry odometry, double myAngle, AngleUnit unit) {
        Point myPos = odometry.getPoint();
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
                    if(!Double.isNaN(myAngle)) {
                        turn = 0.0055 * Math.toDegrees(((myAngle * (unit == AngleUnit.DEGREES ? (Math.PI / 180) : 1)) - Math.toRadians(odometry.getAngle())));
                        if (Math.abs(turn) < 0.1) {
                            turn = 0;
                        }
                    }
                }
            }
        }
        double scaleFactor;
        if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
            scaleFactor = Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        } else {
            scaleFactor = 0.2 / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        }
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
        odometry.update();
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
