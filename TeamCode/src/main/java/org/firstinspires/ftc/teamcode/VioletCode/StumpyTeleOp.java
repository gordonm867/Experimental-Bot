package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name="StumpyTeleOp", group="Linear Opmode")
public class StumpyTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    HardwareStumpy robot = new HardwareStumpy();

    DigitalChannel digitalTouchSensor;
    DigitalChannel digitalRed;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        robot.fright = hardwareMap.dcMotor.get("fright");
        robot.fright.setPower(robot.frightpower);

        robot.fleft = hardwareMap.dcMotor.get("fleft");
        robot.fright.setPower(robot.frightpower);

        robot.rright = hardwareMap.dcMotor.get("rright");
        robot.fright.setPower(robot.rrightpower);

        robot.rleft = hardwareMap.dcMotor.get("rleft");
        robot.fright.setPower(robot.rleftpower);

        robot.lift = hardwareMap.dcMotor.get("lift");
        robot.lift.setPower(robot.liftpower);

        robot.conveyor = hardwareMap.dcMotor.get("conveyor");
        robot.conveyor.setPower(robot.conveyorpower);

        robot.intake1 = hardwareMap.dcMotor.get("intake1");
        robot.intake1.setPower(robot.intake1power);

        robot.intake2 = hardwareMap.dcMotor.get("intake2");
        robot.intake2.setPower(robot.intake1power);

        robot.grabber = hardwareMap.servo.get("grabber");
        robot.grabber.setPosition(robot.grabberopen);

        robot.mover = hardwareMap.servo.get("mover");
        robot.mover.setPosition(robot.moverin);

        robot.foundation = hardwareMap.servo.get("foundation");
        robot.foundation.setPosition(robot.foundationup);

        robot.capstone = hardwareMap.servo.get("capstone");
        robot.capstone.setPosition(robot.capstoneup);

        robot.fleft.setDirection(DcMotor.Direction.REVERSE);
        robot.rleft.setDirection(DcMotor.Direction.REVERSE);
        robot.intake1.setDirection(DcMotor.Direction.REVERSE);

        robot.fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fright.setPower(0);
        robot.fleft.setPower(0);
        robot.rright.setPower(0);
        robot.rleft.setPower(0);
        robot.conveyor.setPower(0);
        robot.lift.setPower(0);
        robot.intake1.setPower(0);
        robot.intake2.setPower(0);

        digitalTouchSensor = hardwareMap.get(DigitalChannel.class, "switch");
        digitalTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        digitalRed = hardwareMap.get(DigitalChannel.class, "red");
        digitalRed.setMode(DigitalChannel.Mode.INPUT);

        robot.fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // GamePad 1
            double drive = -gamepad1.left_stick_y*0.6;
            double turn = gamepad1.right_stick_x*0.5;
            double strafe = gamepad1.left_stick_x;

            if (gamepad1.dpad_down) {
                drive -= 0.3;
            } else if (gamepad1.dpad_up) {
                drive += 0.3;
            }
            if (gamepad1.dpad_left) {
                strafe -= 0.3;
            } else if (gamepad1.dpad_right) {
                strafe += 0.3;
            }

            robot.frightpower = Range.clip(drive - turn - strafe, -0.8, 0.8);
            robot.fleftpower = Range.clip(drive + turn + strafe, -0.8, 0.8);
            robot.rrightpower = Range.clip(drive - turn + strafe, -0.8, 0.8);
            robot.rleftpower = Range.clip(drive + turn - strafe, -0.8, 0.8);


            // Send calculated power to wheels
            robot.fright.setPower(robot.frightpower);
            robot.fleft.setPower(robot.fleftpower);
            robot.rright.setPower(robot.rrightpower);
            robot.rleft.setPower(robot.rleftpower);

            //buttons
            if (gamepad1.x) {
                robot.foundation.setPosition(robot.foundationdown);
            }
            if (gamepad1.a) {
                robot.foundation.setPosition(robot.foundationup);
            }
            if (gamepad1.y) {
                robot.intake1.setPower(0.0);
                robot.intake2.setPower(0.0);
                robot.conveyor.setPower(0.0);
            }
            if (digitalTouchSensor.getState() == false) {
                telemetry.addData("Digital Switch", "Pressed");
                robot.setintake(0);
                robot.grabber.setPosition(robot.grabberclose);
            } else if (gamepad1.right_bumper && digitalTouchSensor.getState() == true) {
                telemetry.addData("Digital Switch", "Not Pressed");
                robot.intake1.setPower(1);
                robot.intake2.setPower(1);
                robot.conveyor.setPower(1);
                robot.intake1.setDirection(DcMotor.Direction.FORWARD);
                robot.intake2.setDirection(DcMotor.Direction.REVERSE);
                robot.conveyor.setDirection(DcMotor.Direction.FORWARD);
                robot.grabber.setPosition(robot.grabberopen);
            }
            if (gamepad1.left_bumper) {
                robot.intake1.setPower(1);
                robot.intake2.setPower(1);
                robot.conveyor.setPower(1);
                robot.intake1.setDirection(DcMotor.Direction.REVERSE);
                robot.intake2.setDirection(DcMotor.Direction.FORWARD);
                robot.conveyor.setDirection(DcMotor.Direction.REVERSE);
            }
            if (gamepad1.right_trigger > 0.3) {
                robot.grabber.setPosition(robot.grabberclose);
            }
            if (gamepad1.left_trigger > 0.3) {
                robot.grabber.setPosition(robot.grabberopen);
            }

            //GamePad 2

            //lift manual control
            double lift = -gamepad2.left_stick_y;
            robot.liftpower = Range.clip(lift, -1.0, 1.0);
            robot.lift.setPower(robot.liftpower);

            //grabber positions
            if (gamepad2.a) {
                robot.grabber.setPosition(robot.grabberclose);
            }
            else if (gamepad2.b) {
                robot.grabber.setPosition(robot.grabberopen);
            }

            //arm reset
            if (gamepad2.x) {
                robot.mover.setPosition(robot.moverin);
                telemetry.addData("Mover", "Moving IN");
                telemetry.addData("Position", robot.mover.getPosition());
            }
            //moving the block out of the robot
            if (gamepad2.y) {
                robot.mover.setPosition(robot.moverout);
                telemetry.addData("Mover", "Moving OUT");
                telemetry.addData("Position", robot.mover.getPosition());
            }
            //encoders
            if (gamepad2.dpad_up) {
                robot.mover.setPosition(robot.movermid);
            }
            if (gamepad2.dpad_down) {
                robot.mover.setPosition(robot.moverout);

            }
            //foundation servo
            if (gamepad2.right_bumper) {
                robot.foundation.setPosition(robot.foundationdown);
            }
            else if (gamepad2.left_bumper) {
                robot.foundation.setPosition(robot.foundationup);
            }
            //capstone servo
            if (gamepad2.left_trigger > 0.5) {
                robot.capstone.setPosition(robot.capstonedown);
            }
            else if (gamepad2.right_trigger > 0.5) {
                robot.capstone.setPosition(robot.capstoneup);
            }

            robot.mover.getPosition();

            telemetry.addData("fleft encoder", robot.fleft.getCurrentPosition());
            telemetry.addData("fright encoder", robot.fright.getCurrentPosition());
            telemetry.addData("rright encoder", robot.rright.getCurrentPosition());
            telemetry.addData("rleft encoder", robot.rleft.getCurrentPosition());

            if (robot.allianceisred()) {
                telemetry.addData("Red/Blue Switch is", "RED");
            } else {
                telemetry.addData("Red/Blue Switch is", "BLUE");
            }
            telemetry.update();
        }

    }
}
