package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.RevBulkData;

@Autonomous(name="RightPark",group="Scrimmage")
@Disabled
public class NoOdometryRedAuto extends LinearOpMode {

    private TrashHardware robot = TrashHardware.getInstance();
    private Drivetrain wheels = new Drivetrain(Drivetrain.State.OFF);
    private double angleOffset = 1;

    private double x = 5.25;
    private double y = -3;

    private OpenCvCamera phoneCam;
    private int skystone = 1;

    private boolean needclamp = false;

    public void runOpMode() throws InterruptedException {

        /* Initialize Hardware */
        robot.init(hardwareMap);
        robot.unlockFoundation();
        Globals.MAX_SPEED = 0.8;
        Globals.START_THETA = 180;
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        resetAllEncoders();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        DetectionPipeline pipeline = new DetectionPipeline();
        phoneCam.setPipeline(pipeline);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        while(!pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }

        /* Initialize Subsystems */
        robot.enabled = true;
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            skystone = pipeline.getBestCol();
            telemetry.addData("Skystone", skystone == 0 ? "Left" : skystone == 1 ? "Center" : "Right");
            telemetry.update();
        }
        waitForStart();
        phoneCam.closeCameraDevice();
        wheels.setState(Drivetrain.State.ON);
        robot.openClamp();
        if(skystone == 0) {
            robot.openClamp();
            robot.lift.setTargetPosition(1000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.6);
            needclamp = true;
            runToPoint(new Point(3.7, -2.3));
            needclamp = false;
            robot.moveClamp(0);
            robot.lift.setTargetPosition(-150);
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            while(robot.lift.isBusy() && opModeIsActive()) {}
            robot.lift.setTargetPosition(-500);
            while(robot.lift.isBusy() && opModeIsActive()) {}
            robot.closeClamp();
            sleep(500);
            runBackToPoint(new Point(3.75, -2.3));
            runToPoint(new Point(3.75, 3));
            robot.lift.setTargetPosition(150);
            runToPoint(new Point(3, 3));
            robot.openClamp();
            sleep(200);
            runBackToPoint(new Point(3.75, 3));
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(3.75, -4.3));
            runToPoint(new Point(2.5, -4.3));
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            robot.lift.setTargetPosition(-150);
            while(opModeIsActive() && robot.lift.isBusy()) {}
            robot.lift.setTargetPosition(-500);
            while(opModeIsActive() && robot.lift.isBusy()) {}
            robot.closeClamp();
            sleep(200);
            runBackToPoint(new Point(3.75, -5));
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(3.75, 3.5));
            robot.lift.setTargetPosition(150);
            runToPoint(new Point(2, 3.5));
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            robot.openClamp();
            robot.unlockFoundation();
            sleep(200);
            runBackToPoint(new Point(4, 4.5));
            robot.lockFoundation();
            sleep(200);
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(4, 1));
            //robot.setDrivePower(-1, 1, -1, 1);
            //sleep(1000);
        }
        else if(skystone == 1) {
            robot.openClamp();
            robot.lift.setTargetPosition(1000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.6);
            needclamp = true;
            runToPoint(new Point(3.7, -3));
            needclamp = false;
            robot.moveClamp(0);
            robot.lift.setTargetPosition(-150);
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            while(robot.lift.isBusy() && opModeIsActive()) {}
            robot.lift.setTargetPosition(-500);
            while(robot.lift.isBusy() && opModeIsActive()) {}
            robot.closeClamp();
            sleep(500);
            runToPoint(new Point(3.45, -3));
            runBackToPoint(new Point(3.72, -3));
            runToPoint(new Point(3.72, 3));
            robot.lift.setTargetPosition(150);
            runToPoint(new Point(3, 3));
            robot.openClamp();
            sleep(200);
            runBackToPoint(new Point(3.5, 3));
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(3.5, -5));
            runToPoint(new Point(2.5, -5));
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            robot.lift.setTargetPosition(-150);
            while(opModeIsActive() && robot.lift.isBusy()) {}
            robot.lift.setTargetPosition(-500);
            while(opModeIsActive() && robot.lift.isBusy()) {}
            robot.closeClamp();
            sleep(200);
            runToPoint(new Point(2.25, -5));
            runBackToPoint(new Point(3.75, -5));
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(3.75, 3.5));
            robot.lift.setTargetPosition(150);
            runToPoint(new Point(2, 3.5));
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            robot.openClamp();
            robot.unlockFoundation();
            sleep(200);
            runBackToPoint(new Point(4, 4.5));
            robot.lockFoundation();
            sleep(200);
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(4, 1));
            //robot.setDrivePower(-1, 1, -1, 1);
            //sleep(1000);
        }
        else {
            robot.openClamp();
            robot.lift.setTargetPosition(1000);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.6);
            needclamp = true;
            runToPoint(new Point(3.7, -3.6));
            needclamp = false;
            robot.moveClamp(0);
            robot.lift.setTargetPosition(-150);
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            while(robot.lift.isBusy() && opModeIsActive()) {}
            robot.lift.setTargetPosition(-500);
            while(robot.lift.isBusy() && opModeIsActive()) {}
            robot.closeClamp();
            sleep(500);
            runBackToPoint(new Point(3.75, -3.6));
            robot.lift.setTargetPosition(150);
            runToPoint(new Point(3.75, 3));
            runToPoint(new Point(3, 3));
            robot.openClamp();
            sleep(200);
            robot.lift.setTargetPosition(-100);
            runBackToPoint(new Point(3.75, 3));
            runToPoint(new Point(3.75, -5));
            runToPoint(new Point(3, -5.2));
            turn(Functions.normalize(-robot.getAngle() - 26.5650511771));
            robot.lift.setTargetPosition(-150);
            while(opModeIsActive() && robot.lift.isBusy()) {}
            robot.lift.setTargetPosition(-500);
            while(opModeIsActive() && robot.lift.isBusy()) {}
            robot.closeClamp();
            sleep(200);
            runBackToPoint(new Point(3.75, -5.2));
            runToPoint(new Point(3.75, 3.5));
            robot.lift.setTargetPosition(150);
            runToPoint(new Point(2, 3.5));
            turn(Functions.normalize(-robot.getAngle() + Globals.START_THETA));
            robot.openClamp();
            robot.unlockFoundation();
            sleep(200);
            runBackToPoint(new Point(4, 4.5));
            robot.lockFoundation();
            sleep(200);
            robot.lift.setTargetPosition(-100);
            runToPoint(new Point(4, 1));
        }
    }

    /**
     * Turn a certain, specified amount
     * @param angle Amount to turn
     */
    public void turn(double angle) {
        if(Math.abs(angle) < angleOffset || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double angleIntended;
        double robotAngle;
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotAngle = robot.getAngle();
        angleIntended = Functions.normalize(robotAngle + angle);
        boolean first = true;
        double lastError = 0;
        long lastTime = System.currentTimeMillis();
        do {
            double error = Functions.normalize(angleIntended - robot.getAngle());
            while (opModeIsActive() && Math.abs(error) > angleOffset) {
                error = Functions.normalize(angleIntended - robot.getAngle());
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
                double Kp = 0.0325 / 1.5f;
                double Kd = -0.0125 / 1.5f;
                double pow = (Kp * error) + (Kd * deriv);
                pow = Math.max(Math.abs(pow), 0.15) * Math.signum(pow);
                robot.setDrivePower(pow, pow, -pow, -pow);
                first = false;
                lastError = error;
                lastTime = System.currentTimeMillis();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        while(Math.abs(robot.getAngle() - angleIntended) > angleOffset);
        telemetry.addData("Error", Math.abs(robot.getAngle() - angleIntended));
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

    /**
     * Reset drive encoders and sets them to RUN_WITHOUT_ENCODERS mode
     */
    private void resetEncoders() { // Reset encoder values and set encoders to "run using encoder" mode
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Reset drive encoders and sets them to RUN_WITHOUT_ENCODERS mode
     */
    private void resetAllEncoders() { // Reset encoder values and set encoders to "run using encoder" mode
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Get the direction in which a certain motor is traveling
     * @param motor Motor on which to perform check
     * @return The direction of the motor (-1 = backwards; 1 = forward; 0 = stationary)
     */
    private double getPower(DcMotor motor) {
        try {
            double power = (Math.abs(motor.getPower()) / motor.getPower()) * (Math.abs(motor.getTargetPosition()) - Math.abs(motor.getCurrentPosition())) / 500.0;
            if(Math.abs(power) >= 0.1) {
                return(Range.clip(power, -1, 1));
            }
            else if(power != 0){
                return(0.1 * (power < 0 ? -1 : 1));
            }
            else {
                return 0;
            }
        }
        catch(Exception p_exception) {
            return 0;
        }
    }

    /**
     * Move backwards to a certain point linearly using encoders to assess accuracy
     * @param newPoint Point to which to travel
     */
    private void runBackToPoint(Point newPoint) {
        if(!opModeIsActive()) {
            return;
        }
        Point currPoint = new Point(x, y);
        double angle;
        try {
            angle = currPoint.angle(newPoint, AngleUnit.DEGREES);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = Functions.normalize(angle - robot.getAngle() + 180);
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance);
        }
        int distance = -currPoint.distance(newPoint);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance / 1500.0));
        // while(!gamepad1.a && opModeIsActive()) {}
        x = newPoint.getX();
        y = newPoint.getY();
    }

    /**
     * Move to a certain point linearly using encoders to assess accuracy
     * @param newPoint Point to which to travel
     */
    private void runToPoint(Point newPoint) {
        if(!opModeIsActive()) {
            return;
        }
        Point currPoint = new Point(x, y);
        double angle;
        try {
            angle = currPoint.angle(newPoint, AngleUnit.DEGREES);
        }
        catch(Exception p_exception) {
            angle = 90;
        }
        double turnDistance = angle - robot.getAngle();
        if(turnDistance > 180) {
            turnDistance -= 360;
        }
        if(turnDistance < -180) {
            turnDistance += 360;
        }
        if(turnDistance != 0) {
            turn(turnDistance);
        }
        int distance = currPoint.distance(newPoint);
        encoderMovePreciseTimed(distance, 1, Math.abs(distance / 1500.0));
        // while(!gamepad1.a && opModeIsActive()) {}
        x = newPoint.getX();
        y = newPoint.getY();
    }

    /**
     * Move a certain distance using drive encoders
     * @param pos Distance to move (in encoder ticks)
     * @param speed Maximum speed for movement
     * @param timeLimit Maximum time to spend on movement
     */
    private void encoderMovePreciseTimed(int pos, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        speed = 0.4;
        RevBulkData data = robot.bulkReadTwo();
        if(pos == 0 && timeLimit >= 0.1) {
            pos = -((data.getMotorCurrentPosition(robot.rf) - data.getMotorCurrentPosition(robot.lf) + data.getMotorCurrentPosition(robot.lb) - data.getMotorCurrentPosition(robot.rb)) / 4);
        }
        resetEncoders();
        if(opModeIsActive()) {
            if (opModeIsActive() && robot.rb != null && robot.rf != null && robot.lb != null && robot.lf != null) {
                robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rb.setTargetPosition(pos);
                robot.rf.setTargetPosition(pos);
                robot.lf.setTargetPosition(pos);
                robot.lb.setTargetPosition(pos);
                double startAngle = robot.getAngle();
                ElapsedTime delta = new ElapsedTime();
                double Kp = 0.0215 / 1.2f;
                double Ki = 0;
                double Kd = 0.01 / 2f;
                double i = 0;
                double lastError = 0;
                double maxSpeed = Globals.MAX_SPEED;
                ElapsedTime limitTest = new ElapsedTime();
                try {
                    robot.setDrivePower(Math.signum(pos) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed);
                } catch (Exception p_exception) {
                    robot.setDrivePower(speed, speed, speed, speed);
                }
                while((Math.abs(((data.getMotorCurrentPosition(robot.rf) - data.getMotorCurrentPosition(robot.lf) + data.getMotorCurrentPosition(robot.lb) - data.getMotorCurrentPosition(robot.rb)) / 4)) <= Math.abs(pos)) && opModeIsActive()) {
                    if(needclamp && !robot.lift.isBusy()) {
                        robot.moveClamp(0.95);
                        needclamp = false;
                    }
                    data = robot.bulkReadTwo();
                    double error = Math.abs(data.getMotorCurrentPosition(robot.rb) - robot.rb.getTargetPosition()) / 500.0;
                    if(error <= 0.25) {
                        error = 0.25;
                    }
                    if(!(speed >= maxSpeed) && !(error <= speed)) {
                        speed += 0.05;
                    }
                    else {
                        speed = Math.min(error, Math.abs(maxSpeed));
                    }
                    if(Math.abs(robot.getAngle() - startAngle) >= 7) {
                        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        double angleError = robot.getAngle() - startAngle;
                        try {
                            if(Math.abs(angleError) > 180 && (Math.abs(robot.getAngle()) / robot.getAngle()) != (Math.abs(startAngle) / startAngle)) {
                                angleError += angleError > 0 ? -360 : 360;
                            }
                        }
                        catch (Exception p_exception) {} // If an error happens, that means that either our current angle or initial angle was zero, so the error calculation should be accurate anyway
                        double right = getPower(robot.rb);
                        double left = getPower(robot.lf);
                        try {
                            robot.setDrivePower((Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed);
                        }
                        catch(Exception p_exception) {
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        double deltaError = angleError - lastError;
                        i += delta.time() * deltaError;
                        double changePower = (Kp * angleError) + (Ki * i) + (Kd * deltaError / delta.time());
                        right += changePower;
                        left -= changePower;
                        double max = Math.max(Math.abs(right), Math.max(Math.abs(left), speed));
                        right /= Math.abs(max);
                        left /= Math.abs(max);
                        robot.setDrivePower(left, left, right, right);
                        lastError = angleError;
                        delta.reset();
                    }
                    else {
                        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        try {
                            robot.setDrivePower((Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed, (Math.signum(pos)) * speed);
                        } catch (Exception p_exception) {
                            robot.setDrivePower(speed, speed, speed, speed);
                        }
                    }
                }
                if (limitTest.time() > timeLimit) {
                    robot.rb.setTargetPosition((robot.rb.getCurrentPosition()));
                    robot.rf.setTargetPosition((robot.rf.getCurrentPosition()));
                    robot.lb.setTargetPosition((robot.lb.getCurrentPosition()));
                    robot.lf.setTargetPosition((robot.lf.getCurrentPosition()));
                }
                robot.setDrivePower(0, 0, 0, 0);
            }
        }
        else {
            throw new IllegalStateException();
        }
    }

}
