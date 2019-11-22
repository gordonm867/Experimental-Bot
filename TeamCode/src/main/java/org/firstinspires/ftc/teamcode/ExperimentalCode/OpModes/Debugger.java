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
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.FoundationMover;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Subsystem;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name = "Debugger", group = "Scrimmage")
public class Debugger extends MyOpMode {

    private     TrashHardware           robot       = TrashHardware.getInstance();
    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();

    private     BoxLift boxlift;
    private     Clamp                   clamp;
    private     Drivetrain              drive;
    private     Extension               extension;
    private     FoundationMover         foundation;
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
        for (Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
    }

    public void loopOp() {
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.in1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.in2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.ex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.liftbox(Globals.boxNeutral);
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        odometry.update(data);
        try {
            telemetry.addData("Pose estimate", odometry.getPoint());
        }
        catch(Exception p_exception) {
            telemetry.addData("Pose estimate", "null");
        }
        try {
            telemetry.addData("Angle estimate", robot.getAngle());
            telemetry.addData("Angle estimate 2", robot.getXAngle());
            telemetry.addData("Angle estimate 3", robot.getYAngle());
        }
        catch(Exception p_exception) {
            telemetry.addData("Angle estimates", "null");
        }
        try {
            telemetry.addData("lb", data2.getMotorCurrentPosition(robot.lb));
        }
        catch(Exception p_exception) {
            telemetry.addData("lb", "null");
        }
        try {
            telemetry.addData("lf", data2.getMotorCurrentPosition(robot.lf));
        }
        catch(Exception p_exception) {
            telemetry.addData("lf", "null");
        }
        try {
            telemetry.addData("rb", data2.getMotorCurrentPosition(robot.rb));
        }
        catch(Exception p_exception) {
            telemetry.addData("rb", "null");
        }
        try {
            telemetry.addData("rf", data2.getMotorCurrentPosition(robot.rf));
        }
        catch(Exception p_exception) {
            telemetry.addData("rf", "null");
        }
        try {
            telemetry.addData("in1", data.getMotorCurrentPosition(robot.in1));
        }
        catch(Exception p_exception) {
            telemetry.addData("in1", "null");
        }
        try {
            telemetry.addData("in2", data.getMotorCurrentPosition(robot.in2));
        }
        catch(Exception p_exception) {
            telemetry.addData("in1", "null");
        }
        try {
            telemetry.addData("in2", data.getMotorCurrentPosition(robot.ex));
        }
        catch(Exception p_exception) {
            telemetry.addData("ex", "null");
        }
        try {
            telemetry.addData("lw", data.getMotorCurrentPosition(robot.lift));
        }
        catch(Exception p_exception) {
            telemetry.addData("lw", "null");
        }
        try {
            telemetry.addData("cl", robot.clamp.getPosition());
        }
        catch(Exception p_exception) {
            telemetry.addData("cl", "null");
        }
        try {
            telemetry.addData("mc", robot.moveClamp.getPosition());
        }
        catch(Exception p_exception) {
            telemetry.addData("mc", "null");
        }
        try {
            telemetry.addData("fm", robot.mover.getPosition());
        }
        catch(Exception p_exception) {
            telemetry.addData("fm", "null");
        }
        try {
            telemetry.addData("fl", robot.boxlift.getPosition());
        }
        catch(Exception p_exception) {
            telemetry.addData("fl", "null");
        }
        telemetry.update();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, data, data2);
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
