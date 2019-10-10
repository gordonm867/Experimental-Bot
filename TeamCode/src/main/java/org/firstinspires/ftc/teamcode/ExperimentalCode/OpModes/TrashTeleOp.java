package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;

import java.util.ArrayList;

@TeleOp(name = "TrashTeleOp", group = "Trash")
public class TrashTeleOp extends MyOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private ArrayList<Subsystem> subsystems = new ArrayList<>();

    private Point myPoint;

    public void initOp() {
        subsystems.add(new Drivetrain(Subsystem.State.ON));
        subsystems.add(new Clamp(Clamp.State.CLOSED));
        subsystems.add(Odometry.getInstance(robot));
        robot.init(hardwareMap);
        robot.enabled = true;
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot);
        }
        telemetry.addData("Angle", ((((Odometry)subsystems.get(2)).getAngle() + 180) % 360) - 180);
        telemetry.addData("x", ((Odometry)subsystems.get(2)).getX() + " --> " + robot.getHOmniPos());
        telemetry.addData("y", -((Odometry)subsystems.get(2)).getY() + " --> " + robot.getVOmniPos());
        telemetry.addData("Odometry", ((Odometry)subsystems.get(2)).isUpdating());
        telemetry.update();
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
