package org.firstinspires.ftc.teamcode.ExperimentalCode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Hardware.TrashHardware;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.LocalizationPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

public class StoneLocalizer implements Subsystem {

    State state;
    OpenCvWebcam webcam;
    LocalizationPipeline pipeline;
    ArrayList<Point> stonePoints = new ArrayList<>();

    public StoneLocalizer(HardwareMap hardwareMap) {
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "cam"));
        webcam.openCameraDevice();
        pipeline = new LocalizationPipeline();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        state = State.ON;
    }

    public StoneLocalizer(HardwareMap hardwareMap, State state) {
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "cam"));
        webcam.openCameraDevice();
        pipeline = new LocalizationPipeline();
        webcam.setPipeline(pipeline);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        this.state = state;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, TrashHardware robot, RevBulkData data1, RevBulkData data2, Odometry odometry) {
        update();
    }

    public void update() {
        stonePoints.clear();
        if(pipeline.isProc && state == State.ON) {
            stonePoints = pipeline.rects;
            for(int x = 0; x < stonePoints.size(); x++) {

            }
        }
    }

    public void setState(State state) {
        this.state = state;
    }
}
