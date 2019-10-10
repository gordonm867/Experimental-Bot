package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

public interface Subsystem {
    enum State{
        ON,
        OFF
    };
    void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot);
    void setState(Subsystem.State newState);
}
