package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

@Config // For use with dashboard
public class Lift implements Subsystem {

    Subsystem.State parentstate; // Parent state (ON/OFF)
    State state; // Child state (enumerated below)

    public static double ACCEL_CONSTANT = 0.05; // Value of constant jerk

    double param = 0.25; // Acceleration value
    double power = 0; // Lift power

    boolean wait = false; // Are we done waiting for the clamp?
    boolean moved = false; // Are we done moving the lift down?
    boolean apressed = false; // Is the "a" button pressed?
    boolean goingToTarget = false; // Is the lift actively going to its target level?
    boolean in = true; // Is the extension in or out?
    boolean bumperPressed = false; // Is the bumper pressed?
    public boolean goingToFakeTarget = false; // Is the lift actively doing a weird automation thing?

    boolean left = false; // Is the left dpad pressed?
    boolean right = false; // Is the right dpad pressed?

    boolean first = true; // Is this the first iteration?

    double time = 0; // Variable to track time elapsed
    double target = 0; // Target lift encoder position

    public int level = 0; // Target lift level
    public int fakeTarget = 0; // Target lift encoder position for weird automation thing

    public boolean isErred = false; // Is the lift encoder working?

    /**
     * All possible child states for Lift
     */
    public enum State {
        LIFTING,
        WAITING,
        AUTOMATED,
        STILL_AUTOMATED,
        IDLE
    }

    /**
     * Constructor for the Lift with default Lift child state
     * @param parentstate ON/OFF
     */
    public Lift(Subsystem.State parentstate) {
        this.parentstate = parentstate;
        this.state = State.IDLE;
    }

    /**
     * Constructor for the Lift with no default state
     * @param parentstate ON/OFF
     * @param state Lift child state
     */
    public Lift(Subsystem.State parentstate, State state) {
        this.parentstate = parentstate;
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
    //   • GAMEPAD1.X (INTAKE)
    //   • GAMEPAD1.Y (OUTTAKE)
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
    //   • GAMEPAD2.X (TOGGLE BOX)

    /**
     * Interpret gamepad values and current circumstances to figure out how to properly power the lift
     * @param gamepad1 Primary gamepad
     * @param gamepad2 Secondary gamepad
     * @param robot Hardware class
     * @param data1 Bulk read for Expansion Hub 2
     * @param data2 Bulk read for Expansion Hub 3
     */
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        double liftspeed = -gamepad2.right_stick_y * ((gamepad2.dpad_up || gamepad2.dpad_down) ? 0.25 : 1);
        if (liftspeed == 0 && ((gamepad2.dpad_up || gamepad2.dpad_down))) {
            if (gamepad2.dpad_up) {
                liftspeed = 0.25;
            } else {
                liftspeed = -0.25;
            }
        }
        if(first && robot.lift != null) {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift(0);
            first = false;
        }
        if(liftspeed != 0 || (state != State.AUTOMATED && state != State.WAITING && state != State.STILL_AUTOMATED)) {
            target = Math.min(-500, -335 * level);
            if(in && robot.lift != null && data1 != null && gamepad2.left_bumper && !bumperPressed) {
                fakeTarget = -500;
                goingToFakeTarget = true;
                bumperPressed = true;
                in = false;
            }
            else if(robot.lift != null && data1 != null && gamepad2.left_bumper && !bumperPressed) {
                fakeTarget = -350;
                goingToFakeTarget = true;
                bumperPressed = true;
                in = true;
            }
            state = liftspeed == 0 ? State.IDLE : State.LIFTING;
            if(robot.lift != null) {
                wait = false;
                moved = false;
            }
            if(gamepad2.dpad_left && !left) {
                level--;
                if(level < 0) {
                    level = 0;
                }
                left = true;
            }
            if(left && !gamepad2.dpad_left) {
                left = false;
            }
            if(gamepad2.dpad_right && !right) {
                level++;
                if(level > 8) {
                    level = 0;
                }
                right = true;
            }
            if(right && !gamepad2.dpad_right) {
                right = false;
            }
            if(gamepad2.x) {
                goingToTarget = true;
            }
            if(!gamepad2.left_bumper && bumperPressed) {
                bumperPressed = false;
            }
            if(robot.lift != null && data1 != null) {
                try {
                    double currentpos = data1.getMotorCurrentPosition(robot.lift);
                    if(target - 10 <= currentpos && target + 10 >= currentpos) {
                        goingToTarget = false;
                    }
                }
                catch(Exception p_exception) {
                    isErred = true;
                    goingToTarget = false;
                }
                try {
                    double currentpos = data1.getMotorCurrentPosition(robot.lift);
                    if(fakeTarget - 10 <= currentpos && fakeTarget + 10 >= currentpos) {
                        goingToFakeTarget = false;
                    }
                }
                catch(Exception p_exception) {
                    isErred = true;
                    goingToFakeTarget = false;
                }
            }
            if(liftspeed != 0) {
                goingToTarget = false;
            }
            if(liftspeed != 0) {
                goingToFakeTarget = false;
            }
            try {
                if (data1 != null && goingToTarget && liftspeed == 0 && robot.lift != null && Math.abs(data1.getMotorCurrentPosition(robot.lift)) < Math.abs(target)) {
                    double error = Math.abs(data1.getMotorCurrentPosition(robot.lift) - target);
                    liftspeed = Math.max(0.3, 1 * (error / 150f));
                }
                else if(data1 != null && goingToTarget && liftspeed == 0 && robot.lift != null) {
                    double error = Math.abs(data1.getMotorCurrentPosition(robot.lift) - target);
                    liftspeed = Math.min(-0.2, -1 * (error / 150f));
                }
            }
            catch(Exception p_exception) {
                isErred = true;
            }
            try {
                if (data1 != null && goingToFakeTarget && liftspeed == 0 && robot.lift != null && Math.abs(data1.getMotorCurrentPosition(robot.lift)) < Math.abs(fakeTarget)) {
                    double error = Math.abs(data1.getMotorCurrentPosition(robot.lift) - fakeTarget);
                    liftspeed = Math.max(0.3, 1 * (error / 150f));                }
                else if(data1 != null && goingToFakeTarget && liftspeed == 0 && robot.lift != null) {
                    double error = Math.abs(data1.getMotorCurrentPosition(robot.lift) - fakeTarget);
                    liftspeed = Math.min(-0.2, -1 * (error / 150f));
                }
            }
            catch(Exception p_exception) {
                isErred = true;
            }
            liftspeed = adjust(Range.clip(liftspeed, -1, 1));
            if (liftspeed != 0 && robot.lift != null) {
                param += ACCEL_CONSTANT;
                liftspeed = Range.clip(liftspeed, power - param, power + param);
                robot.lift(-liftspeed);
                power = -liftspeed;
            }
            else {
                robot.lift(0);
                power = 0;
            }
            if (power == 0) {
                param = 0;
            }
            if(gamepad2.a && !apressed && !isErred) {
                level++;
                if(level > 8) {
                    level = 0;
                }
                state = State.WAITING;
                time = System.currentTimeMillis();
                apressed = true;
            }
            if(apressed && !gamepad2.a) {
                apressed = false;
            }
        }
        if(state == State.WAITING && System.currentTimeMillis() - time >= 100) {
            state = State.AUTOMATED;
            if(robot.lift != null && data1 != null) {
                target = data1.getMotorCurrentPosition(robot.lift) - 150;
            }
        }
        if(state == State.AUTOMATED && robot.lift != null && !checkLift(data1, robot, target) && data1 != null) {
            liftspeed = 0.6 * Math.signum(data1.getMotorCurrentPosition(robot.lift) - target);
            param += ACCEL_CONSTANT;
            liftspeed = Range.clip(liftspeed, power - param, power + param);
            robot.lift(-liftspeed);
            power = -liftspeed;
        }
        if(state == State.AUTOMATED && robot.lift != null && checkLift(data1, robot, target)) {
            robot.lift(0);
            state = State.STILL_AUTOMATED;
            moved = false;
            robot.moveClamp(Clamp.MIN);
            wait = true;
            time = System.currentTimeMillis();
        }
        if(wait && state == State.STILL_AUTOMATED && System.currentTimeMillis() - time >= 4000) {
            wait = false;
            target = -350;
            moved = true;
        }
        if(data1 != null && state == State.STILL_AUTOMATED && moved && !checkLift(data1, robot, target)) {
            liftspeed = Math.signum(data1.getMotorCurrentPosition(robot.lift));
            if(liftspeed == 1) {
                liftspeed = 0;
                target = data1.getMotorCurrentPosition(robot.lift);
            }
            param += ACCEL_CONSTANT;
            liftspeed = Range.clip(liftspeed, power - param, power + param);
            robot.lift(-liftspeed);
            power = -liftspeed;
        }
        if(data1 != null && state == State.STILL_AUTOMATED && moved && robot.lift != null && checkLift(data1, robot, target)) {
            robot.lift(0);
            state = State.IDLE;
        }
    }

    /**
     * Set the Lift's state to ON/OFF
     * @param newstate New desired Lift state (ON/OFF)
     */
    public void setState(Subsystem.State newstate) {
        parentstate = newstate;
    }

    /**
     * Enable square-root driving on the lift
     * @param varToAdjust Lift speed
     * @return Adjusted lift speed for square root curve
     */
    private double adjust(double varToAdjust) { // Square-root driving
        if (varToAdjust < 0) {
            varToAdjust = -Math.sqrt(-varToAdjust);
        } else {
            varToAdjust = Math.sqrt(varToAdjust);
        }
        return varToAdjust;
    }

    /**
     * Check whether the lift is at its target position
     * @param data1 Bulk read data
     * @param robot Hardware
     * @param targ Lift target position
     * @return Boolean representative of whether or not the lift is within a negligible distance of the target
     */
    public boolean checkLift(RevBulkData data1, TrashHardware robot, double targ) {
        boolean toTarg = true;
        try {
            double currentpos = data1.getMotorCurrentPosition(robot.lift);
            if(targ - 10 <= currentpos && targ + 10 >= currentpos) {
                toTarg = false;
            }
        }
        catch(Exception p_exception) {
            isErred = true;
            toTarg = false;
        }
        return !toTarg;
    }
}

/*
package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class Lift implements Subsystem {

    Subsystem.State parentstate;
    State state;

    public static double ACCEL_CONSTANT = 0.05;
    double param = 0.05;
    double power = 0;

    public enum State {
        LIFTING,
        IDLE
    }

    public Lift(Subsystem.State parentstate) {
        this.parentstate = parentstate;
        this.state = State.IDLE;
    }

    public Lift(Subsystem.State parentstate, State state) {
        this.parentstate = parentstate;
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
    //   • GAMEPAD1.X (INTAKE)
    //   • GAMEPAD1.Y (OUTTAKE)
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
    //   • GAMEPAD2.X (TOGGLE BOX)
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        double liftspeed = -gamepad2.right_stick_y * ((gamepad2.dpad_up || gamepad2.dpad_down) ? 0.25 : 1);
        if(liftspeed == 0 && ((gamepad2.dpad_up || gamepad2.dpad_down))) {
            if(gamepad2.dpad_up) {
                liftspeed = 0.25;
            }
            else {
                liftspeed = -0.25;
            }
        }
        liftspeed = adjust(Range.clip(liftspeed, -1, 1));
        if (liftspeed != 0 && robot.lift != null) {

            param += ACCEL_CONSTANT;
            liftspeed = Range.clip(liftspeed, power - param, power + param);
            robot.lift(-liftspeed);
            power = -liftspeed;
        }
        else if(robot.lift != null && gamepad2.b && !gamepad2.start) {
            robot.lift.setTargetPosition(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift(1);
        }
        else {
            if(robot.lift != null && robot.lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                robot.lift(0);
            }
            else if(robot.lift != null && !robot.lift.isBusy()) {
                robot.lift(0);

            }
        }
    }

    public void setState(Subsystem.State newstate) {
        parentstate = newstate;
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
 */