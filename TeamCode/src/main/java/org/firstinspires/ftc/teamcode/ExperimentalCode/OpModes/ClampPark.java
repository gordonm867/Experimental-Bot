package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;

@Autonomous(name="ClampPark",group="Parking")
public class ClampPark extends MyOpMode {

    TrashHardware robot = TrashHardware.getInstance();

    public void initOp() {
        robot.init(hardwareMap);
        robot.moveClamp(Clamp.MIN);
        robot.ex.setTargetPosition(500);
        robot.ex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ex.setPower(0.3);
    }

    public void startOp() {
        long time = System.currentTimeMillis();
        robot.ex.setPower(0);
        robot.ex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive() && Math.abs(time - System.currentTimeMillis()) <= 25000) {}
        robot.moveClamp(Clamp.MAX);
    }

    public void loopOp() {}
}
