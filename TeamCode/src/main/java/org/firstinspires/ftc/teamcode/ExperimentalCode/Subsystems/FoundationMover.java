package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class FoundationMover implements Subsystem {
    private Subsystem.State parState;
    private State state;

    boolean changed = false;

    public FoundationMover(State state) {
        this.state = state;
        parState = Subsystem.State.ON;
    }

    public enum State {
        UNLOCKED,
        LOCKED
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
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        if(parState != Subsystem.State.OFF) {
            if((gamepad1.a && !gamepad1.start) && !changed) {
                changed = true;
                if(state == State.LOCKED) {
                    state = State.UNLOCKED;
                    robot.unlockFoundation();
                }
                else {
                    state = State.LOCKED;
                    robot.lockFoundation();
                }
            }
            if(!(gamepad1.a && !gamepad1.start) && changed) {
                changed = false;
            }
        }
    }

    public void update(TrashHardware robot) {
        if(state == State.LOCKED && parState == Subsystem.State.ON) {
            robot.lockFoundation();
        }
        else if(parState == Subsystem.State.ON) {
            robot.unlockFoundation();
        }
    }

    public void setState(Subsystem.State newState) {
        this.parState = newState;
    }
}
