package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.openftc.revextensions2.RevBulkData;

public interface Subsystem {
    /**
     * Define base states for all subsystems
     */
    enum State{
        ON,
        OFF
    }
    void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData dataOne, RevBulkData dataTwo); // All subsystems need an update() method which figures out what to do based on gamepad inputs and uses robot hardware to do it
    void setState(Subsystem.State newState); // All subsystems need a way of changing state between ON and OFF
}
