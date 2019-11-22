package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class Clamp implements Subsystem {

    boolean bumperPressed = false;

    private State state;
    private Subsystem.State parState;
    private double pos = 0.420;

    boolean changed = false;

    public enum State {
        OPEN,
        CLOSED
    }

    public Clamp(State state) {
        this.state = state;
        parState = Subsystem.State.ON;
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
    //   • GAMEPAD2.X (TOGGLE BOX)
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        if(parState == Subsystem.State.ON) {
            if ((gamepad2.left_bumper || gamepad2.right_bumper) && state == State.OPEN && !bumperPressed) {
                bumperPressed = true;
                state = State.CLOSED;
            } else if ((gamepad2.left_bumper || gamepad2.right_bumper) && state == State.CLOSED && !bumperPressed) {
                bumperPressed = true;
                state = State.OPEN;
            }
            if (!(gamepad2.left_bumper || gamepad2.right_bumper) && bumperPressed) {
                bumperPressed = false;
            }
            if (state == State.OPEN) {
                robot.openClamp();
            } else {
                robot.closeClamp();
            }
            //double trigger = adjust(gamepad2.right_trigger - gamepad2.left_trigger);
            if(gamepad2.right_trigger != 0 && robot.clampRotate != null) {
                robot.clampRotate.setPosition(Globals.clampNeutral);
            }
            if(gamepad2.a && robot.clampRotate != null) {
                robot.clampRotate.setPosition(0.75);
            }
            double trigger = 0;
            if(gamepad2.right_trigger != 0 && !changed) {
                changed = true;
                if(Math.abs(robot.moveClamp.getPosition() - Globals.clampPlace) <= 0.1) {
                    trigger = -1;
                }
                else {
                    trigger = 1;
                }
            }
            if(changed && gamepad2.right_trigger == 0) {
                changed = false;
            }
            pos = Range.clip(trigger, -1, 1);
            robot.moveClampTele(pos);
        }
    }

    public void update(TrashHardware robot) {
        if(parState == Subsystem.State.ON) {
            if(state == State.OPEN) {
                robot.openClamp();
            }
            else {
                robot.closeClamp();
            }
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

    public void setState(State newState) {
        this.state = newState;
    }

    public void setState(Subsystem.State newState) {
        this.parState = newState;
    }

    public Subsystem.State getParentState() {
        return parState;
    }

    public State getState() {
        return state;
    }
}
