package org.firstinspires.ftc.teamcode.ExperimentalCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems.Odometry;

@TeleOp(name = "TrashTeleOp", group = "Trash")
public class TrashTeleOp extends MyOpMode {

    private Odometry odometry;
    private TrashHardware robot = TrashHardware.getInstance();
    private Drivetrain dt = new Drivetrain(Drivetrain.State.DRIVING);

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

    public void loopOp() {
        dt.update(gamepad1, gamepad2, robot);
        odometry.update();
        double time = System.nanoTime();
        double dTime = (time - lastTime) / (Math.pow(10, 9));
        lastTime = time;
        if(robot.gyro != null) {
            xVel += robot.gyro.getGravity().xAccel * dTime * 3.28084;
            yVel += robot.gyro.getGravity().yAccel * dTime * 3.28084;
            x += xVel * dTime;
            y += yVel * dTime;
            myPoint = new Point(x, y);
        }
        telemetry.addData("Angle", ((((odometry.getAngle() + 180) % 360) + 360) % 360) - 180);
        telemetry.addData("x", odometry.getX() + " --> " + robot.getHOmniPos());
        telemetry.addData("y", -odometry.getY() + " --> " + robot.getVOmniPos());
        telemetry.addData("Accelerometer Estimate", myPoint);
        telemetry.addData("Odometry", odometry.isUpdating());
        telemetry.update();
    }

    public void stopOp() {
        dt.setState(Drivetrain.State.STOPPED);
        robot.enabled = false;
    }

}
