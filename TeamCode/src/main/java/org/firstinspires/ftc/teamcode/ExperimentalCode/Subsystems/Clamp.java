package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

public class Clamp {

    boolean bumperPressed = false;

    private State state;
    private double pos = 0.420;

    public enum State {
        OPEN,
        CLOSED
    }
    public Clamp(State state) {
        this.state = state;
    }

    public void update(TrashHardware robot, Gamepad gamepad1, Gamepad gamepad2) {
        if((gamepad2.left_bumper || gamepad2.right_bumper) && state == State.OPEN && !bumperPressed) {
            bumperPressed = true;
            state = State.CLOSED;
        }
        else if((gamepad2.left_bumper || gamepad2.right_bumper) && state == State.CLOSED && !bumperPressed) {
            bumperPressed = true;
            state = State.OPEN;
        }
        if((gamepad2.left_bumper || gamepad2.right_bumper) && bumperPressed) {
            bumperPressed = false;
        }
        if(state == State.OPEN) {
            robot.openClamp();
        }
        else {
            robot.closeClamp();
        }
        if(gamepad2.dpad_right) {
            pos -= 0.01;
        }
        if(gamepad2.dpad_left) {
            pos += 0.01;
        }
        pos = Range.clip(pos, 0, 1);
        robot.moveClamp(pos);
    }
}
