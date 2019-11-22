package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class BoxLift implements Subsystem {

    private State state;
    private Subsystem.State parState;

    boolean xchanged = false;

    public enum State {
        DOWN,
        NEUTRAL
    }
    public BoxLift(State state) {
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
    //   • GAMEPAD2.A (INTAKE)
    //   • GAMEPAD2.B (OUTTAKE)
    //   • GAMEPAD2.X (TOGGLE BOX)
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        if(parState == Subsystem.State.ON) {
            if(gamepad2.x && !xchanged) {
                this.state = state == State.DOWN ? State.NEUTRAL : State.DOWN;
                xchanged = true;
            }
            if(!gamepad2.x) {
                xchanged = false;
            }
            if(state == State.DOWN) {
                robot.liftbox(Globals.boxDown);
            }
            else {
                robot.liftbox(Globals.boxNeutral);
                // -1654
            }
        }
    }

    public void update(TrashHardware robot) {
        if(parState == Subsystem.State.ON) {
            if(state == State.NEUTRAL) {
                robot.liftbox(0.45);
            }
            else {
                robot.liftbox(0.38);
            }
        }
    }

    public void setState(State newState) {
        this.state = newState;
    }

    public void setState(Subsystem.State newState) {
        this.parState = newState;
    }
}
