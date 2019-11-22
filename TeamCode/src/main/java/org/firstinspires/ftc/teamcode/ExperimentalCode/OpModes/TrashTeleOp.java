package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.BoxLift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name = "WoodTeleOp", group = "Trash")
public class TrashTeleOp extends MyOpMode {

    private     TrashHardware           robot      = TrashHardware.getInstance();
    private     ArrayList<Subsystem>    subsystems = new ArrayList<>();

    private     BoxLift                 boxlift;
    private     Clamp                   clamp;
    private     Drivetrain              drive;
    private     Extension               extension;
    private     FoundationMover         foundation;
    private     Intake                  intake;
    private     Lift                    lift;
    private     Odometry                odometry;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        boxlift = new BoxLift(BoxLift.State.NEUTRAL);
        clamp = new Clamp(Clamp.State.CLOSED);
        drive = new Drivetrain(Subsystem.State.OFF);
        extension = new Extension(Extension.State.IDLE);
        foundation = new FoundationMover(FoundationMover.State.UNLOCKED);
        intake = new Intake(Intake.State.IDLE);
        lift = new Lift(Subsystem.State.ON);
        odometry = Odometry.getInstance(robot);

        robot.enabled = true;
        subsystems.add(boxlift);

        subsystems.add(clamp);
        subsystems.add(drive);
        subsystems.add(extension);
        subsystems.add(foundation);
        subsystems.add(intake);
        subsystems.add(lift);
        subsystems.add(odometry);
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, data, data2);
        }
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
