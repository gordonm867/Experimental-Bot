package org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class TrashHardware {
    /* Declare OpMode members */
    public              BNO055IMU       gyro;

    public              DcMotor         rf;
    public              DcMotor         rb;
    public              DcMotor         lf;
    public              DcMotor         lb;
    private             DcMotor         hOmni;
    private             DcMotor         vOmni;

    private             ExpansionHubEx  ex2;


    private static      TrashHardware   myInstance      = null;

    public              boolean         enabled         = true;


    /* Constructor */
    public static TrashHardware getInstance() {
        if(myInstance == null) {
            myInstance = new TrashHardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {
        try {
            ex2 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        }
        catch(Exception p_exception) {
            ex2 = null;
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
            lb = hwMap.get(DcMotor.class, "lr");
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
            rb = hwMap.get(DcMotor.class, "rr");
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

        try { // Right front wheel
            hOmni = hwMap.get(DcMotor.class, "h");
            hOmni.setDirection(DcMotor.Direction.FORWARD);
            hOmni.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hOmni.setPower(0);
            hOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            hOmni = null;
        }

        try { // Right front wheel
            vOmni = hwMap.get(DcMotor.class, "v");
            vOmni.setDirection(DcMotor.Direction.FORWARD);
            vOmni.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vOmni.setPower(0);
            vOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            vOmni = null;
        }
    }

    public double getAngle() {
        return (((gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + Globals.START_THETA) + 180) % 360) - 180;
    }

    public RevBulkData bulkRead() {
        if(ex2 != null) {
            return ex2.getBulkInputData();
        }
        return null;
    }

    public void setDrivePower(double leftBackDrivePower, double leftFrontDrivePower, double rightBackDrivePower, double rightFrontDrivePower) { // Send power to wheels
        if(enabled) {
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

    public void resetOmnis() {
        vOmni.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hOmni.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hOmni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getVOmniPos() {
        if(vOmni != null) {
            return vOmni.getCurrentPosition();
        }
        return 0;
    }

    public int getHOmniPos() {
        if(hOmni != null) {
            return -hOmni.getCurrentPosition();
        }
        return 0;
    }
}
