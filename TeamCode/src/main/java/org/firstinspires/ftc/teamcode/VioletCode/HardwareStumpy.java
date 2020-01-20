package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**************************************************************************************************/

@Config
public class HardwareStumpy
{

    /* Public OpMode members. */
    public Servo grabber;
    public double grabberopen = 0.777;
    public double grabberclose = 0.015;

    public Servo mover;
    public double moverin = 0.725;
    public double moverout = 0.587;
    public double movermid = 0.643;

    public Servo foundation;
    public double foundationup = 0.961;
    public double foundationdown = 0.456;

    public Servo capstone;
    public double capstoneup = 1;
    public double capstonedown =0.435;

    public DcMotor fright;
    public double frightpower = 0;

    public DcMotor lift;
    public double liftpower = 0;

    public DcMotor fleft;
    public double fleftpower = 0;

    public DcMotor rright;
    public double rrightpower = 0;

    public DcMotor rleft;
    public double rleftpower = 0;

    public DcMotor conveyor;
    public double conveyorpower = 0.0;

    public DcMotor intake1;
    public double intake1power = 0;

    public DcMotor intake2;
    public double intake2power = 0;

    public double motorspeed = 0;
    public double intake = 0;

    public double strafe = 0;
    public double leftstrafe = 0;
    public double rightstrafe = 0;

    public double skystonedst = 1000;

    /**********************************************************************************************/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* int pos = 0;
     int pos1 = lift.getCurrentPosition();

     int currentposition = pos;*/
    DigitalChannel digitalRed;



    /* Constructor */
    public HardwareStumpy(){

    }

    public void init(HardwareMap hardwareMap) {

        fright = hardwareMap.dcMotor.get("fright");
        fright.setPower(frightpower);
        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fleft = hardwareMap.dcMotor.get("fleft");
        fleft.setPower(fleftpower);
        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rright = hardwareMap.dcMotor.get("rright");
        rright.setPower(rrightpower);
        rright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rleft = hardwareMap.dcMotor.get("rleft");
        rleft.setPower(rleftpower);
        rleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setPower(liftpower);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        conveyor  = hardwareMap.dcMotor.get("conveyor");
        conveyor.setPower(conveyorpower);
        conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake1.setPower(intake1power);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake2= hardwareMap.dcMotor.get("intake2");
        intake2.setPower(intake2power);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber = hardwareMap.servo.get("grabber");
        grabber.setPosition(grabberopen);

        mover = hardwareMap.servo.get("mover");
        mover.setPosition(moverin);

        foundation = hardwareMap.servo.get("foundation");
        foundation.setPosition(foundationup);

        capstone = hardwareMap.servo.get("capstone");
        capstone.setPosition(capstonedown);

        digitalRed = hardwareMap.get(DigitalChannel .class, "red");
        digitalRed.setMode(DigitalChannel.Mode.INPUT);

        fleft.setDirection(DcMotor.Direction.REVERSE);
        rleft.setDirection(DcMotor.Direction.REVERSE);

        fright.setPower(0);
        fleft.setPower(0);
        rright.setPower(0);
        rleft.setPower(0);

        lift.setPower(0);

        conveyor.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }

    void setMotorspeed (double speed) {
        fleft.setPower(speed);
        rleft.setPower(speed);
        fright.setPower(speed);
        rright.setPower(speed);
    }
    void setintake (double speed) {
        intake1.setPower(speed);
        intake2.setPower(speed);
        conveyor.setPower(speed);
    }
    void setLeftstrafe () {

        strafe+=0.3;

        frightpower = Range.clip(-strafe, -0.6, 0.6);
        fleftpower = Range.clip(+strafe, -0.6, 0.6);
        rrightpower = Range.clip(+strafe+0.075, -0.8, 0.8);
        rleftpower = Range.clip(-strafe-0.075, -0.8, 0.8);


        fleft.setPower(fleftpower);
        fright.setPower(frightpower);
        rleft.setPower(rleftpower);
        rright.setPower(rrightpower);
    }
    void setRightstrafe () {
        strafe-=-0.3;

        frightpower = Range.clip(+strafe, -0.6, 0.6);
        fleftpower = Range.clip(-strafe, -0.6, 0.6);
        rrightpower = Range.clip(-strafe+0.075, -0.8, 0.8);
        rleftpower = Range.clip(+strafe-0.075, -0.8, 0.8);


        fleft.setPower(fleftpower);
        fright.setPower(frightpower);
        rleft.setPower(rleftpower);
        rright.setPower(rrightpower);
    }
    void setLeftside (double speed) {
        fleft.setPower(speed);
        rleft.setPower(speed);

    }
    void setRightside (double speed) {
        fright.setPower(speed);
        rright.setPower(speed);
    }
    boolean allianceisred(){
        return !digitalRed.getState();
    }
}
