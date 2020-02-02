package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class MyOpMode extends LinearOpMode {

    public void runOpMode() {
        initOp();
        while(!isStopRequested() && !isStarted()) {
            init_loopOp();
        }
        startOp();
        while(opModeIsActive() && !isStopRequested()) {
            loopOp();
        }
        stopOp();
    }

    public abstract void initOp();
    public void init_loopOp() {}
    public void startOp() {}
    public abstract void loopOp();
    public void stopOp() {}
}