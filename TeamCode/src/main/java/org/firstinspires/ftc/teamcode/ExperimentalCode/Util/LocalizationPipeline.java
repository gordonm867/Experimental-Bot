package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import org.firstinspires.ftc.teamcode.ExperimentalCode.Globals.Globals;
import org.firstinspires.ftc.teamcode.ExperimentalCode.Math.Point;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class LocalizationPipeline extends OpenCvPipeline {
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    Mat myMat = new Mat();
    Mat disp = new Mat();
    Mat filtered = new Mat();
    Mat test = new Mat();

    public boolean isProc = false;

    private ArrayList<MatOfPoint> contlist = new ArrayList<>();
    public ArrayList<Point> rects = new ArrayList<>();

    public int contors = 0;
    int newcontors = 0;

    @Override
    public Mat processFrame(Mat input) {
        newcontors = 0;
        filtered = new Mat();
        disp = new Mat();
        myMat = new Mat();
        input.copyTo(myMat);
        input.copyTo(disp);
        Imgproc.cvtColor(myMat, myMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(myMat, myMat, new Size((Globals.BLUR_CONSTANT * 2) + 1, (Globals.BLUR_CONSTANT * 2) + 1), 0);
        Core.inRange(myMat, new Scalar(Globals.minB, Globals.minG, Globals.minR), new Scalar(Globals.maxB, Globals.maxG, Globals.maxR), filtered);
        contlist.clear();
        Imgproc.findContours(filtered, contlist, test, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<MatOfPoint> toRemove = new ArrayList<>();
        int x = 0;
        for(MatOfPoint contour : contlist) {
            double test = contour.size().height * contour.size().width;
            if(test >= 25) {
                if(contlist.size() > x) {
                    Imgproc.drawContours(disp, contlist, x, new Scalar(128, 0, 128), 2);
                    newcontors++;
                }
            }
            else {
                toRemove.add(contour);
            }
            x++;
        }
        contlist.removeAll(toRemove);
        toRemove.clear();
        rects.clear();
        for(MatOfPoint contour : contlist) {
            rects.add(new Point(Imgproc.boundingRect(contour).x, Imgproc.boundingRect(contour).y));
        }
        contors = newcontors;
        isProc = true;
        return disp;
    }
}