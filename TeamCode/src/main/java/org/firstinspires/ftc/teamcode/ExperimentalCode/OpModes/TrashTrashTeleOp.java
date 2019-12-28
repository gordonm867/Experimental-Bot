package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.OLDBoxLift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.OLDClamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.OLDExtension;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.OLDLift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name = "OLD-WoodTeleOp", group = "Trash")
public class TrashTrashTeleOp extends MyOpMode {

    private     TrashHardware           robot      = TrashHardware.getInstance();
    private     ArrayList<Subsystem>    subsystems = new ArrayList<>();

    private     OLDBoxLift              boxlift;
    private     OLDClamp                clamp;
    private     Drivetrain              drive;
    private     OLDExtension            extension;
    private     Intake                  intake;
    private     OLDLift                 lift;
    private     FoundationMover         mover;
    private     Odometry                odometry;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        Globals.boxDown = 0.395;
        robot.init(hardwareMap);
        boxlift = new OLDBoxLift(OLDBoxLift.State.NEUTRAL);
        clamp = new OLDClamp(OLDClamp.State.CLOSED);
        drive = new Drivetrain(Subsystem.State.OFF);
        extension = new OLDExtension(OLDExtension.State.IDLE);
        mover = new FoundationMover(FoundationMover.State.UNLOCKED);
        intake = new Intake(Intake.State.IDLE);
        lift = new OLDLift(Subsystem.State.ON);
        odometry = Odometry.getInstance(robot);

        robot.liftOdometry();
        robot.unlockFoundation();

        robot.enabled = true;
        subsystems.add(boxlift);
        subsystems.add(clamp);
        subsystems.add(drive);
        subsystems.add(extension);
        subsystems.add(intake);
        subsystems.add(lift);
        subsystems.add(mover);
        subsystems.add(odometry);

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
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
