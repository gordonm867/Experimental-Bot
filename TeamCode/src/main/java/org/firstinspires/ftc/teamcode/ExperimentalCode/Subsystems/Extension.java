package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class Extension implements Subsystem {
    private Subsystem.State parState;
    public State state;

    private boolean xchanged = false;
    private boolean released = true;
    public boolean seconded = true;
    private boolean xsecondchanged = false;
    private boolean aboutToAutomate = false;
    private double time = 0;

    private double target = 100;

    public Extension(State state) {
        this.state = state;
        parState = Subsystem.State.ON;
    }

    public enum State {
        EXTENDING,
        AUTO,
        TRANSITIONING,
        IDLE,
    }

    public void setState(Subsystem.State newState) {
        parState = newState;
    }

    public void setState(State newState) {
        state = newState;
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
    //   • gamepad2.left_bumper (BOX DOWN)
    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        if(gamepad2.left_bumper && !xchanged && state != State.AUTO && seconded) {
            seconded = false;
            released = false;
            aboutToAutomate = true;
            xchanged = true;
            time = System.currentTimeMillis();
        }
        if(!gamepad2.left_bumper && xchanged) {
            xchanged = false;
        }
        if(aboutToAutomate && System.currentTimeMillis() - time >= 200) {
            seconded = false;
            aboutToAutomate = false;
            state = State.AUTO;
        }
        if((state == State.AUTO || aboutToAutomate) && !gamepad2.left_bumper) {
            released = true;
        }
        if(gamepad2.left_bumper && state == State.AUTO && released && !xsecondchanged) {
            state = State.TRANSITIONING;
            xsecondchanged = true;
        }
        if(xsecondchanged && !gamepad2.left_bumper) {
            xsecondchanged = false;
        }
        if(!seconded && !gamepad2.left_bumper && state != State.AUTO) {
            seconded = true;
        }
        /*
        if(parState == Subsystem.State.ON && (gamepad2.left_stick_y != 0 || state != State.AUTO && state != State.TRANSITIONING)) {
            if(gamepad2.left_stick_y == 0 && !(gamepad2.dpad_right || gamepad2.dpad_left)) {
                state = State.IDLE;
            }
            else {
                state = State.EXTENDING;
            }
            if(state == State.IDLE) {
                robot.extend(0);
            }
            else {
                double extend = -gamepad2.left_stick_y * ((gamepad2.dpad_right || gamepad2.dpad_left) ? 0.25 : 1);
                robot.extend(extend);
            }
        }

         */
        if(state == State.AUTO) {
            if(robot.ex != null) {
                robot.ex.setTargetPosition(Globals.EXTEND_POS);
                target = Globals.EXTEND_POS;
                robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ex.setPower(0.75);
            }
        }
        else if(state == State.TRANSITIONING) {
            if(robot.ex != null) {
                target = 0;
                robot.ex.setTargetPosition(0);
                robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ex.setPower(1);
                if(!robot.ex.isBusy()) {
                    state = State.IDLE;
                }
            }
        }
    }
}