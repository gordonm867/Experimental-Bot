package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;

@TeleOp(name = "TrashTeleOp", group = "Trash")
public class TrashTeleOp extends OpMode {

    private Odometry odometry;
    private TrashHardware robot = TrashHardware.getInstance();
    private Drivetrain dt = new Drivetrain(Drivetrain.State.DRIVING);

    @Override
    public void init() {
        robot.init(hardwareMap);
        odometry = Odometry.getInstance(robot);
        robot.enabled = true;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        dt.update(gamepad1, gamepad2, robot);
        odometry.update();
        telemetry.addData("Angle", ((odometry.getAngle() + 180) % 360) - 180);
        telemetry.addData("x", odometry.getX() + " --> " + robot.getHOmniPos());
        telemetry.addData("y", -odometry.getY() + " --> " + robot.getVOmniPos());
        telemetry.addData("Odometry", odometry.isUpdating());
        telemetry.update();
    }

    @Override
    public void stop() {
        dt.setState(Drivetrain.State.STOPPED);
        robot.enabled = false;
    }

}
