package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class Lift implements Subsystem {

    Subsystem.State parentstate;
    State state;

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
        if(liftspeed != 0) {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift(liftspeed);
        }
        else if(gamepad2.b && !gamepad2.start) {
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setTargetPosition(-355);
            robot.lift.setPower(1);
        }
        else {
            if(robot.lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                robot.lift.setPower(0);
            }
            else if(!robot.lift.isBusy()) {
                robot.lift.setPower(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
