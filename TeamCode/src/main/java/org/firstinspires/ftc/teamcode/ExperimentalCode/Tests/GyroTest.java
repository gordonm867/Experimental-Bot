package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;

@TeleOp(name="Gyro Test",group="Tests")
@Disabled
public class GyroTest extends LinearOpMode {
    public void runOpMode() {
        TrashHardware robot = TrashHardware.getInstance();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while(opModeIsActive()) {
            telemetry.addData("Angle", robot.getAngle());
            telemetry.update();
        }
    }
}
