package org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Functions;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.StoneClamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.RGBA;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

/**
 * HUB 2
 *  MOTORS
 *   • ex
 *   • lw
 *   • in1
 *   • in2
 *  SERVOS
 *   •
 *   •
 *   •
 *   •
 *  ANALOG
 *   •
 *   •
 *  I2C 0
 *   • Port 0: g0
 *
 * HUB 3
 *  MOTORS
 *   • lf
 *   • rf
 *   • lb
 *   • rb
 *  SERVOS
 *   • fm1
 *   • cl (clamp open/close)
 *   • mc (clamp slide)
 *   • fm2
 *   • odo
 *   • cap
 *  ANALOG
 */
@Config
public class TrashHardware {
    /* Declare OpMode members */
    public              BNO055IMU       gyro;

    public              ColorSensor     moresense;

    public              DcMotor         rf;
    public              DcMotor         rb;
    public              DcMotor         lf;
    public              DcMotor         lb;
    public              DcMotor         ex;
    public              DcMotor         in1;
    public              DcMotor         in2;
    public              DcMotor         lift;

    public              DistanceSensor  sense;

    public              ExpansionHubEx  ex2;
    public              ExpansionHubEx  ex3;

    public              Servo           clamp;
    public              Servo           stoneBase;
    public              Servo           mover1;
    public              Servo           mover2;
    public              Servo           boxlift;
    public              Servo           moveClamp;
    public              Servo           clampRotate;
    public              Servo           odometry;
    public              Servo           cap;
    public              Servo           claw;


    private static      TrashHardware   myInstance      = null;

    public              boolean         enabled         = true;
    public              boolean         intaking        = false;

    public              double          clampPose       = 0;

    public static       double          cappose         = 0.8;
    public static       double          uncap           = 0;

    public static       double          fm1Open         = 0.440; // OLD: 0.500
    public static       double          fm1Closed       = 0.225; // OLD: 0.150
    public static       double          fm2Open         = 0.070; // OLD: 0.050
    public static       double          fm2Closed       = 0.325; // OLD: 0.400


    /* Constructor */
    public static TrashHardware getInstance() {
        if(myInstance == null) {
            myInstance = new TrashHardware();
        }
        return myInstance;
    }

    /**
     * Initialize robot hardware
     * @param hwMap OpMode's internal HardwareMap
     */
    public void init(HardwareMap hwMap) {
        try {
            ex2 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        }
        catch(Exception p_exception) {
            ex2 = null;
        }
        try {
            ex3 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        }
        catch(Exception p_exception) {
            ex3 = null;
        }
        try { // Gyro
            gyro = hwMap.get(BNO055IMU.class, "g0");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            if(ex2 != null) {
                gyro = LynxOptimizedI2cFactory.createLynxEmbeddedImu(ex2.getStandardModule(), 0);
            }
            gyro.initialize(parameters);
        } catch (Exception p_exception) {
            gyro = null;
        }
        try { // Left rear wheel
            lb = hwMap.get(DcMotor.class, "lb");
            lb.setDirection(DcMotor.Direction.REVERSE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setPower(0);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            lb = null;
        }

        try { // Left front wheel
            lf = hwMap.get(DcMotor.class, "lf");
            lf.setDirection(DcMotor.Direction.REVERSE);
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lf.setPower(0);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            lf = null;
        }

        try { // Right rear wheel
            rb = hwMap.get(DcMotor.class, "rb");
            rb.setDirection(DcMotor.Direction.FORWARD);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setPower(0);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            rb = null;
        }

        try { // Right front wheel
            rf = hwMap.get(DcMotor.class, "rf");
            rf.setDirection(DcMotor.Direction.FORWARD);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setPower(0);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            rf = null;
        }

        try {
            in1 = hwMap.get(DcMotor.class, "in1");
            in1.setDirection(DcMotor.Direction.FORWARD);
            in1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            in1.setPower(0);
            in1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            in1 = null;
        }

        try {
            in2 = hwMap.get(DcMotor.class, "in2");
            in2.setDirection(DcMotor.Direction.REVERSE);
            in2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            in2.setPower(0);
            in2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            in2 = null;
        }

        try {
            lift = hwMap.get(DcMotor.class, "lw");
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            lift = null;
        }

        try {
            clamp = hwMap.get(Servo.class, "cl");
            clamp.setDirection(Servo.Direction.FORWARD);
            openClamp();
        }
        catch(Exception p_exception) {
            clamp = null;
        }

        try {
            cap = hwMap.get(Servo.class, "cap");
            cap.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            cap = null;
        }

        try {
            moveClamp = hwMap.get(Servo.class, "mc");
            moveClamp.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            moveClamp = null;
        }

        try {
            stoneBase = hwMap.get(Servo.class, "bs");
            stoneBase.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            stoneBase = null;
        }

        try {
            claw = hwMap.get(Servo.class, "claw");
            claw.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            claw = null;
        }

        try {
            mover1 = hwMap.get(Servo.class, "fm1");
            mover1.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            mover1 = null;
        }

        try {
            mover2 = hwMap.get(Servo.class, "fm2");
            mover2.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            mover2 = null;
        }

        try {
            odometry = hwMap.get(Servo.class, "odo");
            odometry.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            odometry = null;
        }

        try {
            boxlift = hwMap.get(Servo.class, "fl");
            boxlift.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            boxlift = null;
        }

        try {
            clampRotate = hwMap.get(Servo.class, "ot");
            clampRotate.setDirection(Servo.Direction.FORWARD);
        }
        catch(Exception p_exception) {
            clampRotate = null;
        }

        try { // Right front wheel
            ex = hwMap.get(DcMotor.class, "ex");
            ex.setDirection(DcMotor.Direction.FORWARD);
            ex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ex.setPower(0);
            ex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            ex = null;
        }

        try {
            sense = hwMap.get(DistanceSensor.class, "ins");
            moresense = hwMap.get(ColorSensor.class, "ins");
        }
        catch(Exception p_exception) {
            sense = null;
            moresense = null;
        }

    }

    /**
     * Get robot's angle
     * @return Robot angle
     */
    public double getAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + Globals.START_THETA);
    }

    /**
     * Get robot's angle
     * @return Robot angle
     */
    public double getYAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle + Globals.START_THETA);
    }

    /**
     * Get robot's angle
     * @return Robot angle
     */
    public double getXAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle + Globals.START_THETA);
    }

    /**
     * Get bulk data
     * @return REV Hub bulk data
     */
    public RevBulkData bulkRead() {
        if(ex2 != null) {
            return ex2.getBulkInputData();
        }
        return null;
    }

    /**
     * Get secondary bulk data
     * @return REV Hub bulk data
     */
    public RevBulkData bulkReadTwo() {
        if(ex3 != null) {
            return ex3.getBulkInputData();
        }
        return null;
    }

    /**
     * Drive with specified wheel powers
     * @param leftBackDrivePower Power [-1, 1] for left rear drive wheel
     * @param leftFrontDrivePower Power [-1, 1] for left front drive wheel
     * @param rightBackDrivePower Power [-1, 1] for right rear drive wheel
     * @param rightFrontDrivePower Power [-1, 1] for right front drive wheel
     */
    public void setDrivePower(double leftBackDrivePower, double leftFrontDrivePower, double rightBackDrivePower, double rightFrontDrivePower) { // Send power to wheels
        if(enabled) {
            /*
            if(Math.abs(leftBackDrivePower) < Globals.MIN_SPEED && leftBackDrivePower != 0) {
                leftBackDrivePower = Globals.MIN_SPEED * Math.signum(leftBackDrivePower);
            }
            if(Math.abs(leftFrontDrivePower) < Globals.MIN_SPEED && leftFrontDrivePower != 0) {
                leftFrontDrivePower = Globals.MIN_SPEED * Math.signum(leftFrontDrivePower);
            }
            if(Math.abs(rightBackDrivePower) < Globals.MIN_SPEED && rightBackDrivePower != 0) {
                rightBackDrivePower = Globals.MIN_SPEED * Math.signum(rightBackDrivePower);
            }
            if(Math.abs(rightFrontDrivePower) < Globals.MIN_SPEED && rightFrontDrivePower != 0) {
                rightFrontDrivePower = Globals.MIN_SPEED * Math.signum(rightFrontDrivePower);
            }

             */
            if (lb != null) {
                leftBackDrivePower = Range.clip(leftBackDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                lb.setPower(leftBackDrivePower);
            }
            if (lf != null) {
                leftFrontDrivePower = Range.clip(leftFrontDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                lf.setPower(leftFrontDrivePower);
            }
            if (rb != null) {
                rightBackDrivePower = Range.clip(rightBackDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                rb.setPower(rightBackDrivePower);
            }
            if (rf != null) {
                rightFrontDrivePower = Range.clip(rightFrontDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                rf.setPower(rightFrontDrivePower);
            }
        }
    }

    /**
     * Open the clamp system for intaking
     */
    public void openClamp() {
        if(clamp != null) {
            clamp.setPosition(Globals.clampOpen);
        }
    }

    /**
     * Close the clamp system to grip a block
     */
    public void closeClamp() {
        if(clamp != null) {
            clamp.setPosition(Globals.clampClose);
        }
    }

    public void rotateClamp(double newPos) {
        if(clampRotate != null) {
            double pos = clampRotate.getPosition();
            clampRotate.setPosition(Range.clip(pos + newPos, 0, 1));
        }
    }

    /**
     * Move the clamp system around
     * @param newPos New position to which to move clamp
     */
    public void moveClamp(double newPos) {
        if(moveClamp != null) {
            clampPose = Range.clip(newPos, 0, 1);
            moveClamp.setPosition(clampPose);
        }
    }

    public void liftbox(double newpos) {
        if(boxlift != null) {
            boxlift.setPosition(newpos);
        }
        /*
        if(Math.abs(newpos - Globals.boxNeutral) < Math.abs(newpos - Globals.boxDown)) {
            intaking = false;
        }
        else {
            intaking = true;
        }
         */
    }

    /**
     * Lock foundation mover onto foundation
     */
    public void lockFoundation() {
        if(mover1 != null && mover2 != null) {
            mover1.setPosition(fm1Closed);
            mover2.setPosition(fm2Closed);
        }
    }

    /**
     * Release foundation mover
     */
    public void unlockFoundation() {
        if(mover1 != null && mover2 != null) {
            mover1.setPosition(fm1Open);
            mover2.setPosition(fm2Open);
        }
    }

    public void liftOdometry() {
        odometry.setPosition(1);
    }

    public void dropOdometry() {
        odometry.setPosition(0);
    }

    /**
     * Move extension
     * @param speed Speed at which to move extension (will be clipped to interval [-1, 1]
     */
    public void extend(double speed) {
        if(ex != null) {
            if(ex.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                ex.setPower(0);
                ex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            ex.setPower(Range.clip(speed, -1, 1));
        }
    }

    /**
     * Lift arm system
     * @param speed Speed at which to lift
     */
    public void lift(double speed) {
        if(lift != null) {
            lift.setPower(Range.clip(speed, -1, 1));
        }
    }

    public void moveStoneBase(double newPos) {
        if(stoneBase != null) {
            stoneBase.setPosition(newPos);
        }
    }

    public void setInPower(double power) {
        if(in1 != null && in2 != null) {
            power = Range.clip(power, -1, 1);
            in1.setPower(-power);
            in2.setPower(-power);
        }
    }

    public void setInPowerTest(double power) {
        if(in1 != null && in2 != null) {
            power = Range.clip(power, -1, 1);
            in1.setPower(power);
            in2.setPower(power / 2);
        }
    }

    /**
     * Reset omni encoders
     */
    public void resetOmnis() {
        in1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        in2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        in1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        in2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get current encoder reading of vertically aligned omni
     * @return Encoder reading
     */
    public int getVOmniPos(RevBulkData rev) {
        if(in1 != null) {
            return rev.getMotorCurrentPosition(in1);
        }
        return 0;
    }

    public double getDistance(DistanceUnit unit) {
        if(sense != null) {
            return sense.getDistance(unit);
        }
        return Double.NaN;
    }

    public void cap() {
        if(cap != null) {
            cap.setPosition(cappose);
        }
    }

    public void uncap() {
        if(cap != null) {
            cap.setPosition(uncap);
        }
    }

    public RGBA getRGB() {
        if(moresense != null) {
            return new RGBA (moresense.red(), moresense.green(), moresense.blue(), moresense.alpha());
        }
        return null;
    }

    /**
     * Get current encoder reading of horizontally aligned omni
     * @return Encoder reading
     */
    public int getHOmniPos(RevBulkData rev) {
        if(in2 != null) {
            return -rev.getMotorCurrentPosition(in2);
        }
        return 0;
    }

    public void closeClaw() {
        if(claw != null) {
            claw.setPosition(StoneClamp.CLOSED);
        }
    }

    public void openClaw() {
        if(claw != null) {
            claw.setPosition(StoneClamp.OPEN);
        }
    }

    public boolean hasBlock() {
        double distance = sense.getDistance(DistanceUnit.CM);
        return(!Double.isNaN(distance) && distance < 13);
    }

    /*
    if(target.getIntakePower() < 0 && index < 15) {
            if(data.getMotorCurrentPosition(robot.lift) < 300) {
                robot.lift.setPower(-0.25);
            }
            else {
                robot.lift.setPower(0);
            }
        }
        else if(index > 0 && targets.get(index - 1).getIntakePower() < 0) {
            if(data.getMotorCurrentPosition(robot.lift) > -300) {
                robot.lift.setPower(1);
            }
        }
        else {
            robot.lift.setPower(0);
        }
     */
}