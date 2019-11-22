package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

@TeleOp(name="Tuner",group="GOFTests")
public class ServoTuner extends MyOpMode {
    TrashHardware robot = TrashHardware.getInstance();
    Servo servo = null;
    public void initOp() {
        robot.init(hardwareMap);
        servo = robot.boxlift;
    }
    public void loopOp() {
        servo.setPosition(Globals.boxDown);
    }
}
