package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Clamp;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;

@TeleOp(name = "TrashTeleOp", group = "Trash")
public class TrashTeleOp extends LinearOpMode {

    private Odometry odometry;
    private TrashHardware robot = TrashHardware.getInstance();
    private Drivetrain dt = new Drivetrain(Drivetrain.State.DRIVING);
    private Clamp clamp = new Clamp(Clamp.State.OPEN);

    private double lastTime = System.nanoTime();

    private double xVel = 0;
    private double yVel = 0;

    private double x;
    private double y;

    private Point myPoint;

    public void initOp() {
        robot.init(hardwareMap);
        odometry = Odometry.getInstance(robot);
        x = odometry.getX();
        y = odometry.getY();
        robot.enabled = true;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void initLoop() {

    }

    public void loopOp() {
        dt.update(gamepad1, gamepad2, robot);

        odometry.update();
        double time = System.nanoTime();
        double dTime = time - lastTime;
        lastTime = time;
        if(robot.gyro != null) {
            xVel += robot.gyro.getGravity().xAccel * dTime;
            yVel += robot.gyro.getGravity().yAccel * dTime;
            x += xVel * dTime;
            y += yVel * dTime;
            myPoint = new Point(x, y);
        }
        telemetry.addData("Angle", ((odometry.getAngle() + 180) % 360) - 180);
        telemetry.addData("x", odometry.getX() + " --> " + robot.getHOmniPos());
        telemetry.addData("y", -odometry.getY() + " --> " + robot.getVOmniPos());
        telemetry.addData("Accelerometer Estimate", myPoint);
        telemetry.addData("Odometry", odometry.isUpdating());
        telemetry.update();
    }

    public void startOp() {

    }

    public void stopOp() {
        dt.setState(Drivetrain.State.STOPPED);
        robot.enabled = false;
    }

    public void runOpMode() {
        initOp();
        while(!isStarted() && !isStopRequested()) {
            initLoop();
        }
        startOp();
        while(opModeIsActive()) {
            loopOp();
        }
        stopOp();
    }

}
