package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public class Capper implements Subsystem {
    private Subsystem.State parState;
    private State state;

    boolean changed = false;

    public Capper(State state) {
        this.state = state;
        parState = Subsystem.State.ON;
    }

    public enum State {
        CAPPING,
        IDLE
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2) {
        if(gamepad1.y) {
            robot.cap();
        }
        if(gamepad1.x) {
            robot.uncap();
        }
    }

    public void setState(Subsystem.State newState) {
        this.parState = newState;
    }
}