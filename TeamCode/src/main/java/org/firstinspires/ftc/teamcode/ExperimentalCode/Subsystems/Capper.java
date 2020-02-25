package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class Capper implements Subsystem {
    private Subsystem.State parState;
    private State state;

    boolean changed = false;
    boolean pressed = false;
    boolean capped = false;

    double time = System.currentTimeMillis();

    public Capper(State state) {
        this.state = state;
        parState = Subsystem.State.ON;
    }

    public enum State {
        CAPPING,
        IDLE
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2, Odometry odometry) {
        if(gamepad1.y && !pressed) {
            pressed = true;
            robot.cap();
            time = System.currentTimeMillis();
            capped = true;
        }
        if(!gamepad1.y) {
            pressed = false;
        }
        if(capped && Math.abs(System.currentTimeMillis() - time) >= 750) {
            double rand = (Math.round(Math.random() * 2) % 2) + 1;
            if(rand % 2 == 1) {
                robot.uncap();
            }
            else {
                robot.cap();
            }
        }
        if(gamepad1.x) {
            robot.uncap();
            capped = false;
        }
    }

    public void setState(Subsystem.State newState) {
        this.parState = newState;
    }
}