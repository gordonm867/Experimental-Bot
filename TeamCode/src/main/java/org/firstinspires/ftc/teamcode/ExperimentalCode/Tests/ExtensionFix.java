package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

@TeleOp(name="ExtensionFix")
@Disabled
public class ExtensionFix extends LinearOpMode {
    private TrashHardware robot = TrashHardware.getInstance();
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        waitForStart();
        while(!gamepad1.x && !gamepad1.y) {}
        while(gamepad1.x) {
            robot.extend(-0.5);
        }
        while(gamepad1.y) {
            robot.extend(0.5);
        }
    }
}
