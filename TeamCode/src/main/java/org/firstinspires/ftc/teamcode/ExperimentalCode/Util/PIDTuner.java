package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;

import java.util.ArrayList;

@TeleOp(name="PIDTuner",group="Util")
public class PIDTuner extends MyOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Odometry odometry;
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);
    private double angleOffset = 1;
    private double tuneCoeff = 0.75;
    double bestTime = Double.MAX_VALUE;
    boolean zerozero = false;
    double bestcoeff = 0;
    ArrayList<Double> results = new ArrayList<>();

    public void initOp() {
        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Globals.START_THETA = 0;
        Globals.START_X = 2;
        Globals.START_Y = 2;
        odometry = Odometry.getInstance(robot);
    }

    public void startOp() {
        robot.enabled = true;
        wheels.setState(Drivetrain.State.ON);
        //tune(0.1, 1, 0.1);
        //results.add(tuneCoeff);
        tuneCoeff = 0.01;
        bestTime = Double.MAX_VALUE;
        bestcoeff = 0;
        tune(0.01, 2, 0.01);
        results.add(tuneCoeff);
        tuneCoeff = 0.01;
        bestTime = Double.MAX_VALUE;
        bestcoeff = 0;
        tune(0.01, 3, 0.01);
        results.add(tuneCoeff);
        tuneCoeff = results.get(1);
        bestTime = Double.MAX_VALUE;
        bestcoeff = 0;
        tune(0.01, 4, 0.01);
        results.add(tuneCoeff);
        wheels.setState(Drivetrain.State.OFF);
        robot.enabled = false;
    }

    public void loopOp() {
        telemetry.addData("Strafe coefficient", results.get(0));
        telemetry.addData("Turn Kp coefficient", results.get(3));
        telemetry.addData("Turn Kd coefficient", results.get(2));
        telemetry.update();
    }

    public void tune(double rate, int option, double initial) {
        if(rate < initial / Math.pow(2, 4)) {
            return;
        }
        long time = System.currentTimeMillis();
        if(option == 1) {
            Point target = zerozero ? new Point(2, 2) : new Point(0, 0);
            zerozero = !zerozero;
            double displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            while (Math.abs(displacement) > 0.15 && !(System.currentTimeMillis() - time >= 5000)) {
                wheels.update(tuneCoeff, robot, target, odometry, Double.NaN, AngleUnit.DEGREES);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            }
            robot.setDrivePower(0, 0, 0, 0);
            sleep(2000);
        }
        else if(option == 2) {
            turnTuneP(90 * Math.signum(Math.random() - 0.5));
            sleep(2000);
        }
        else if(option == 3) {
            turnTuneD(90 * Math.signum(Math.random() - 0.5), results.get(1));
            sleep(2000);
        }
        else {
            turnTunePAgain(90 * Math.signum(Math.random() - 0.5), results.get(2));
            sleep(2000);
        }
        long newTime = System.currentTimeMillis();
        odometry.update();
        double x = newTime - time;
        if(bestTime > x) {
            bestTime = x;
            bestcoeff = tuneCoeff;
            tuneCoeff += rate;
            tune(rate, option, initial);
        }
        else if(bestTime + 500 > x) {
            tuneCoeff += rate;
            tune(rate, option, initial);
        }
        else {
            tuneCoeff = bestcoeff;
        }
        tune(rate / 2, option, initial);
    }

    public void turnTuneP(double angle) {
        if(Math.abs(angle) < angleOffset || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double angleIntended;
        double robotAngle;
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = odometry.getAngle();
        angleIntended = Functions.normalize(robotAngle + angle);
        boolean first = true;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();
        do {
            double error = Functions.normalize(angleIntended - odometry.getAngle());
            while (opModeIsActive() && Math.abs(error) > angleOffset) {
                odometry.update();
                error = Functions.normalize(angleIntended - odometry.getAngle());
                if (Math.abs(error + 360) < Math.abs(error)) {
                    error += 360;
                }
                if (Math.abs(error - 360) < Math.abs(error)) {
                    error -= 360;
                }
                double delta = lastError - error;
                double deltaTime = System.currentTimeMillis() - lastTime;
                double deriv = 0;
                if(!first) {
                    deriv = delta / deltaTime;
                }
                double Kp = tuneCoeff;
                double Kd = 0;
                double pow = (Kp * error) + (Kd * deriv);
                pow = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                robot.setDrivePower(pow, pow, -pow, -pow);
                first = false;
                lastError = error;
                lastTime = System.currentTimeMillis();
            }
            robot.setDrivePower(0, 0, 0, 0);
            odometry.update();
            sleep(30);
        }
        while(Math.abs(odometry.getAngle() - angleIntended) > angleOffset);
        telemetry.addData("Error", Math.abs(odometry.getAngle() - angleIntended));
        telemetry.update();
        resetEncoders();
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
    }

    public void turnTuneD(double angle, double Kp) {
        if(Math.abs(angle) < angleOffset || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double angleIntended;
        double robotAngle;
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = odometry.getAngle();
        angleIntended = Functions.normalize(robotAngle + angle);
        boolean first = true;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();
        do {
            double error = Functions.normalize(angleIntended - odometry.getAngle());
            while (opModeIsActive() && Math.abs(error) > angleOffset) {
                odometry.update();
                error = Functions.normalize(angleIntended - odometry.getAngle());
                if (Math.abs(error + 360) < Math.abs(error)) {
                    error += 360;
                }
                if (Math.abs(error - 360) < Math.abs(error)) {
                    error -= 360;
                }
                double delta = lastError - error;
                double deltaTime = System.currentTimeMillis() - lastTime;
                double deriv = 0;
                if(!first) {
                    deriv = delta / deltaTime;
                }
                double Kd = -tuneCoeff;
                double pow = (Kp * error) + (Kd * deriv);
                pow = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                robot.setDrivePower(pow, pow, -pow, -pow);
                first = false;
                lastError = error;
                lastTime = System.currentTimeMillis();
            }
            robot.setDrivePower(0, 0, 0, 0);
            odometry.update();
            sleep(30);
        }
        while(Math.abs(odometry.getAngle() - angleIntended) > angleOffset);
        telemetry.addData("Error", Math.abs(odometry.getAngle() - angleIntended));
        telemetry.update();
        resetEncoders();
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
    }

    public void turnTunePAgain(double angle, double Kd) {
        if(Math.abs(angle) < angleOffset || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double angleIntended;
        double robotAngle;
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotAngle = odometry.getAngle();
        angleIntended = Functions.normalize(robotAngle + angle);
        boolean first = true;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();
        do {
            double error = Functions.normalize(angleIntended - odometry.getAngle());
            while (opModeIsActive() && Math.abs(error) > angleOffset) {
                odometry.update();
                error = Functions.normalize(angleIntended - odometry.getAngle());
                if (Math.abs(error + 360) < Math.abs(error)) {
                    error += 360;
                }
                if (Math.abs(error - 360) < Math.abs(error)) {
                    error -= 360;
                }
                if(Math.abs(error) < angleOffset) {
                    break;
                }
                double delta = lastError - error;
                double deltaTime = System.currentTimeMillis() - lastTime;
                double deriv = 0;
                if(!first) {
                    deriv = delta / deltaTime;
                }
                double Kp = tuneCoeff;
                double pow = (Kp * error) + (Kd * deriv);
                pow = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                robot.setDrivePower(pow, pow, -pow, -pow);
                first = false;
                lastError = error;
                lastTime = System.currentTimeMillis();

            }
            robot.setDrivePower(0, 0, 0, 0);
            odometry.update();
            sleep(30);
        }
        while(Math.abs(Functions.normalize(odometry.getAngle() - angleIntended)) > angleOffset);
        telemetry.addData("Error", Math.abs(Functions.normalize(odometry.getAngle() - angleIntended)));
        telemetry.update();
        resetEncoders();
        /*
        error = -getAngle() + angleIntended;
        if(error > 180) {
            error -= 180;
        }
        else if(error < -180) {
            error += 180;
        }
        if(Math.abs(error) > 0.1 && time - turnTime.time() > 0.75 && turns <= 1) {
            turn(error, time - turnTime.time());
        }
        else {
            turns = 0;
        }
        */
    }

    private void resetEncoders() { // Reset encoder values and set encoders to "run using encoder" mode
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
