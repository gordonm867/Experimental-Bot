package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

@TeleOp(name="ServoTest",group="Trash")
public class ServoTest extends MyOpMode {
    TrashHardware robot = TrashHardware.getInstance();
    private double pos = 0;
    boolean pressed = false;
    boolean otherpressed = false;

    public void initOp() {
        robot.init(hardwareMap);
    }

    public void loopOp() {
        if(gamepad1.a & !pressed) {
            pos += 0.01;
            pressed = true;
        }
        if(pressed && !gamepad1.a) {
            pressed = false;
        }
        if(gamepad1.b & !otherpressed) {
            pos -= 0.01;
            otherpressed = true;
        }
        if(otherpressed && !gamepad1.b) {
            otherpressed = false;
        }
        robot.moveClamp(pos);
        telemetry.addData("pos", pos);
        telemetry.update();
    }
}
