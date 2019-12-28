package org.firstinspires.ftc.teamcode.ExperimentalCode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.MyOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Util.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="VisionTest",group="GOFTests")
@Disabled
public class VisionTest extends MyOpMode {

    OpenCvWebcam phoneCam;
    private int myCol = 1;

    public void initOp() {
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

        while(!isStarted() && !isStopRequested()) {
            myCol = pipeline.getBestCol();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Skystone", myCol == 0 ? "Left" : myCol == 1 ? "Center" : "Right");
            telemetry.update();
        }
    }

    public void loopOp() {
        telemetry.addData("Finalized Skystone Position", myCol == 0 ? "Left" : myCol == 1 ? "Center" : "Right");
        telemetry.update();
    }
}