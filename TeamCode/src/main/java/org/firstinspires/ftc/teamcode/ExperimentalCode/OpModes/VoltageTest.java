package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.BoxLift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Extension;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name = "Current Draw", group = "Scrimmage")
public class VoltageTest extends MyOpMode {

    private     TrashHardware           robot       = TrashHardware.getInstance();
    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();

    private     BoxLift                 boxlift;
    private     Clamp                   clamp;
    private     Drivetrain              drive;
    private     Extension               extension;
    private     Intake                  intake;
    private     Lift                    lift;
    private     Odometry                odometry;

    private     boolean                 changed     = false;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        boxlift = new BoxLift(BoxLift.State.NEUTRAL);
        clamp = new Clamp(Clamp.State.CLOSED);
        drive = new Drivetrain(Subsystem.State.OFF);
        extension = new Extension(Extension.State.IDLE);
        intake = new Intake(Intake.State.IDLE);
        lift = new Lift(Subsystem.State.ON);
        odometry = Odometry.getInstance(robot);

        robot.enabled = true;
        subsystems.add(boxlift);

        subsystems.add(clamp);
        subsystems.add(drive);
        subsystems.add(extension);
        subsystems.add(intake);
        subsystems.add(lift);
        subsystems.add(odometry);
        for (Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        odometry.update(data);
        try {
            telemetry.addData("lb", ((ExpansionHubMotor)(robot.lb)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("lb", "null");
        }
        try {
            telemetry.addData("lf", ((ExpansionHubMotor)(robot.lf)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("lf", "null");
        }
        try {
            telemetry.addData("rb", ((ExpansionHubMotor)(robot.rb)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("rb", "null");
        }
        try {
            telemetry.addData("rf", ((ExpansionHubMotor)(robot.rf)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("rf", "null");
        }
        try {
            telemetry.addData("in1", ((ExpansionHubMotor)(robot.in1)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("in1", "null");
        }
        try {
            telemetry.addData("in2", ((ExpansionHubMotor)(robot.in2)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("in1", "null");
        }
        try {
            telemetry.addData("ex", ((ExpansionHubMotor)(robot.ex)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("ex", "null");
        }
        try {
            telemetry.addData("lw", ((ExpansionHubMotor)(robot.lift)).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        }
        catch(Exception p_exception) {
            telemetry.addData("lw", "null");
        }
        telemetry.update();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, data, data2, odometry);
        }
        if(gamepad1.b && !changed) {
            odometry.reset();
            changed = true;
        }
        if(!gamepad1.b && changed) {
            changed = false;
        }
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
