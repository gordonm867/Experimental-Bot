package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

public class Drivetrain implements Subsystem {

    private State state;
    private boolean field = false;
    private boolean changed = false;

    private boolean backwards = false;

    public Drivetrain(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public void setState(State state) {
        this.state = state;
    }

    // CONTROLS:
    //  • GAMEPAD1
    //   • GAMEPAD1.LEFT_STICK (DRIVE + STRAFE)
    //   • GAMEPAD1.RIGHT_STICK (TURN)
    //   • GAMEPAD1.LEFT_BUMPER (SLOW LEFT TURN)
    //   • GAMEPAD1.RIGHT_BUMPER (SLOW RIGHT TURN)
    //   • GAMEPAD1.RIGHT_TRIGGER (PROPORTIONAL SLOWDOWN)
    //   • GAMEPAD1.DPAD_UP (SLOW FORWARD)
    //   • GAMEPAD1.DPAD_DOWN (SLOW BACKWARD)
    //   • GAMEPAD1.DPAD_LEFT (SLOW LEFT)
    //   • GAMEPAD1.DPAD_RIGHT (SLOW RIGHT)
    //   • GAMEPAD1.A (TOGGLE FOUNDATION MOVER)
    //   • GAMEPAD1.START + GAMEPAD1.Y (TOGGLE DRIVE ORIENTATION)
    //  • GAMEPAD2
    //   • GAMEPAD2.LEFT_STICK (EXTENSION)
    //   • GAMEPAD2.RIGHT_STICK (LIFT)
    //   • GAMEPAD2.LEFT_BUMPER (TOGGLE CLAMP)
    //   • GAMEPAD2.RIGHT_BUMPER (TOGGLE CLAMP)
    //   • GAMEPAD2.LEFT_TRIGGER (CLAMP LEFT)
    //   • GAMEPAD2.RIGHT_TRIGGER (CLAMP RIGHT)
    //   • GAMEPAD2.DPAD_UP (SLOW LIFT)
    //   • GAMEPAD2.DPAD_DOWN (SLOW RETRACT LIFT)
    //   • GAMEPAD2.DPAD_LEFT (SLOW RETRACT EXTENSION)
    //   • GAMEPAD2.DPAD_RIGHT (SLOW EXTEND)
    //   • GAMEPAD2.A (INTAKE)
    //   • GAMEPAD2.B (OUTTAKE)
    //   • GAMEPAD2.X (BOX DOWN)
    //   • GAMEPAD2.Y (BOX NEUTRAL)
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2, Odometry odometry) {
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double angle = -gamepad1.left_stick_x;
        if(state == State.ON) {
            /* Precision vertical drive */
            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                if (gamepad1.left_stick_y != 0) {
                    drive = drive * 0.1; // Slow down joystick driving
                } else {
                    if (gamepad1.dpad_down) {
                        drive = -0.1; // Slow drive backwards
                    } else {
                        drive = 0.1; // Slow drive forwards
                    }
                }
            }

            /* Precision sideways drive */
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                if (gamepad1.right_stick_x != 0) {
                    angle = angle * 0.15; // Slow down joystick side movement
                } else {
                    if (gamepad1.dpad_left) {
                        angle = -0.15; // Slow leftwards
                    } else {
                        angle = 0.15; // Slow rightwards
                    }
                }
            }

            /* Precision turn */
            if (gamepad1.left_bumper) {
                turn = -0.1; // Slow left turn
            }
            if (gamepad1.right_bumper) {
                turn = 0.1; // Slow right turn
            }
            if(gamepad1.start && gamepad1.y && !changed) {
                field = !field;
                changed = true;
            }
            if(changed && !(gamepad1.start && gamepad1.y)) {
                changed = false;
            }
            if(!field) {
                drive = adjust(drive);
                turn = adjust(turn);
                angle = adjust(angle);
                double scaleFactor;
                //double maxspeed = Math.min(Globals.MAX_SPEED, Math.max(Math.abs(robot.lf.getPower()), Math.max(Math.abs(robot.lb.getPower()), Math.max(Math.abs(robot.rf.getPower()), Math.abs(robot.rb.getPower())))) + 0.2);
                if (Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))) > Globals.MAX_SPEED) {
                    scaleFactor = Globals.MAX_SPEED / (Math.max(Math.abs(drive + turn + angle), Math.abs(drive - turn - angle)));
                } else {
                    scaleFactor = Globals.MAX_SPEED;
                }
                scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
                robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
            }
            else {
                double maxspeed = Math.max(Math.abs(drive + angle), Math.abs(drive - angle));
                double driveangle = Math.atan2(drive, angle);
                double relAngle = Math.toRadians(Math.toDegrees(driveangle) - robot.getAngle());
                drive = Math.cos(relAngle);
                angle = Math.sin(relAngle);
                double scaleFactor = maxspeed / Math.max(Math.abs(drive + angle), Math.abs(drive - angle));
                scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
                robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
            }
        }
        else if (state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(double lr, double lf, double rr, double rf, TrashHardware robot) {
        if(state == State.ON) {
            robot.setDrivePower(lr, lf, rr, rf); // Set motors to values based on gamepad
        }
        else if(state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(ArrayList<Double> powers, TrashHardware robot) {
        if(state == State.ON) {
            robot.setDrivePower(powers.get(0), powers.get(1), powers.get(2), powers.get(3)); // Set motors to values based on gamepad
        }
        else if(state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(double[] powers, TrashHardware robot) {
        if(state == State.ON) {
            robot.setDrivePower(powers[0], powers[1], powers[2], powers[3]); // Set motors to values based on gamepad
        }
        else if(state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(TrashHardware robot, Point target, Odometry odometry, double myAngle, double current, RevBulkData data) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    double error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 1.0) {
                        error = Functions.normalize(myAngle - current);
                        if (Math.abs(error + 360) < Math.abs(error)) {
                            error += 360;
                        }
                        if (Math.abs(error - 360) < Math.abs(error)) {
                            error -= 360;
                        }
                        double Kp = 0.0325;
                        double pow = (Kp * error);
                        turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                    }
                }
                if(Math.abs(displacement) <= (Math.sqrt(2) / 10) || (Math.abs(angle) < 0.001 && Math.abs(drive) < 0.001)) {
                    drive = 0;
                    angle = 0;
                    if(!Double.isNaN(myAngle)) {
                        double error = Functions.normalize(myAngle - current);
                        if(Math.abs(error) >= 1.0) {
                            error = Functions.normalize(myAngle - current);
                            if (Math.abs(error + 360) < Math.abs(error)) {
                                error += 360;
                            }
                            if (Math.abs(error - 360) < Math.abs(error)) {
                                error -= 360;
                            }
                            double Kp = 0.0325;
                            double pow = (Kp * error);
                            turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                        }
                    }
                }
            }
        }
        double scaleFactor;
        if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
            scaleFactor = Math.abs(Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
        } else {
            if(displacement >= 0.5) {
                scaleFactor = Math.abs(Math.max(displacement * 2, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
            }
            else {
                scaleFactor = Math.abs(Math.max(displacement / 2.5, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
            }
        }
        odometry.update(data, current);
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
    }

    public void update(TrashHardware robot, Point target, Odometry odometry, double myAngle, double velocity, double delta, double current, RevBulkData data) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    double error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 1.0) {
                        error = Functions.normalize(myAngle - current);
                        if (Math.abs(error + 360) < Math.abs(error)) {
                            error += 360;
                        }
                        if (Math.abs(error - 360) < Math.abs(error)) {
                            error -= 360;
                        }
                        double Kp = 0.0325;
                        double pow = (Kp * error);
                        turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                    }
                }
                if(Math.abs(displacement) <= (Math.sqrt(2) / 10) || (Math.abs(angle) < 0.001 && Math.abs(drive) < 0.001)) {
                    drive = 0;
                    angle = 0;
                    if(!Double.isNaN(myAngle)) {
                        double error = Functions.normalize(myAngle - current);
                        if(Math.abs(error) >= 1.0) {
                            error = Functions.normalize(myAngle - current);
                            if (Math.abs(error + 360) < Math.abs(error)) {
                                error += 360;
                            }
                            if (Math.abs(error - 360) < Math.abs(error)) {
                                error -= 360;
                            }
                            double Kp = 0.0325;
                            double pow = (Kp * error);
                            turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                        }
                    }
                }
            }
        }
        double scaleFactor;
        if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) { // If our highest speed is greater than one
            backwards = false; // The robot is moving forward
            scaleFactor = Math.abs(Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))))); // Scale speed to max at our maximum speed parameter
        } else {
            if(displacement >= 0.5) { // If we are not within six inches of our target
                backwards = false; // The robot should move forward
                scaleFactor = Math.abs(Math.max(displacement * 2, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))))); // Scale speed to equal double our displacement from our target in feet
            }
            else { // If we have to decelerate
                if(delta < 0 || Math.abs(velocity) < 1) { // If our distance to our target increases or our velocity is sufficiently slow
                    backwards = false; // The robot should move forward
                    scaleFactor = Math.abs((Globals.MIN_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))))); // Move robot at minimum non-stall speed
                }
                else { // If our distance to our target is still decreasing and our velocity is still too high
                    backwards = true;
                    scaleFactor = Range.clip(((-1.25 * velocity) + 0.25), -Globals.MAX_SPEED, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))); // Force the wheels backwards based on the current robot velocity
                }
            }
            if(Math.abs(scaleFactor) < Globals.MIN_SPEED) { // If the scale factor is particularly low
                scaleFactor = Globals.MIN_SPEED * Math.signum(scaleFactor); // Move it higher to ensure that we will not stall the robot
            }
        }
        odometry.update(data, current); // Update the odometry pose estimate
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Power the motors
    }

    public void clupdate(TrashHardware robot, Point target, Odometry odometry, double myAngle, double velocity, double delta, double current, RevBulkData data) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    double error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 1.0) {
                        error = Functions.normalize(myAngle - current);
                        if (Math.abs(error + 360) < Math.abs(error)) {
                            error += 360;
                        }
                        if (Math.abs(error - 360) < Math.abs(error)) {
                            error -= 360;
                        }
                        double Kp = 0.0325;
                        double pow = (Kp * error);
                        turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                    }
                }
                if(Math.abs(displacement) <= (Math.sqrt(2) / 100) || (Math.abs(angle) < 0.00001 && Math.abs(drive) < 0.00001)) {
                    drive = 0;
                    angle = 0;
                    if(!Double.isNaN(myAngle)) {
                        double error = Functions.normalize(myAngle - current);
                        if(Math.abs(error) >= 1.0) {
                            error = Functions.normalize(myAngle - current);
                            if (Math.abs(error + 360) < Math.abs(error)) {
                                error += 360;
                            }
                            if (Math.abs(error - 360) < Math.abs(error)) {
                                error -= 360;
                            }
                            double Kp = 0.0325;
                            double pow = (Kp * error);
                            turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                        }
                    }
                }
            }
        }
        double scaleFactor;
        if(displacement >= 0.5 && Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
            backwards = false;
            scaleFactor = Math.abs(Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
        } else {
            if(displacement >= 0.5) {
                backwards = false;
                scaleFactor = Math.abs(Math.max(displacement * 2, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
            }
            else {
                if(delta < 0 || Math.abs(velocity) < 1) {
                    backwards = false;
                    scaleFactor = Math.abs((Globals.MIN_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))))));
                }
                else {
                    backwards = true;
                    scaleFactor = Range.clip(((-1.25 * velocity) + 0.25), -Globals.MAX_SPEED, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
                }
            }
            if(Math.abs(scaleFactor) < Globals.MIN_SPEED) {
                scaleFactor = Globals.MIN_SPEED * Math.signum(scaleFactor);
            }
        }
        /*
        telemetry.addData("Backwards", backwards);
        telemetry.update();

         */
        odometry.update(data, current);
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
    }

    public double[] calcUpdate(Point target, Odometry odometry, double myAngle, double current) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    double error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 1.0) {
                        error = Functions.normalize(myAngle - current);
                        if (Math.abs(error + 360) < Math.abs(error)) {
                            error += 360;
                        }
                        if (Math.abs(error - 360) < Math.abs(error)) {
                            error -= 360;
                        }
                        double Kp = 0.0325;
                        double pow = (Kp * error);
                        turn = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                    }
                }
                if(Math.abs(displacement) <= (Math.sqrt(2) / 10) || (Math.abs(angle) < 0.001 && Math.abs(drive) < 0.001)) {
                    drive = 0;
                    angle = 0;
                    if(!Double.isNaN(myAngle)) {
                        double error = Functions.normalize(myAngle - current);
                        if(Math.abs(error) >= 1.0) {
                            error = Functions.normalize(myAngle - current);
                            if (Math.abs(error + 360) < Math.abs(error)) {
                                error += 360;
                            }
                            if (Math.abs(error - 360) < Math.abs(error)) {
                                error -= 360;
                            }
                            double Kp = 0.0325;
                            double pow = (Kp * error);
                            turn = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                        }
                    }
                }
            }
        }
        double scaleFactor;
        if(Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))) > 1) {
            scaleFactor = Math.abs(Globals.MAX_SPEED / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
        } else {
            if(displacement >= 0.5) {
                scaleFactor = Math.abs(Math.max(displacement * 2, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
            }
            else {
                scaleFactor = Math.abs(Math.max(displacement / 2.5, Globals.MIN_SPEED) / Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle))))));
            }
        }
        double[] mylist = new double[4];
        mylist[0] = scaleFactor * (drive + turn - angle);
        mylist[1] = scaleFactor * (drive + turn + angle);
        mylist[2] = scaleFactor * (drive - turn + angle);
        mylist[3] = scaleFactor * (drive - turn - angle);
        return mylist;
    }

    /*
    private double adjust(double varToAdjust) { // Square-root driving
        if (varToAdjust < 0) {
            varToAdjust = -Math.sqrt(-varToAdjust);
        } else {
            varToAdjust = Math.sqrt(varToAdjust);
        }
        return varToAdjust;
    }

     */

    public double adjust(double varToAdjust) {
        return (Math.atan(5 * varToAdjust) / Math.atan(5));
    }

}
