/*package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AlternateSamplePipeline extends OpenCvPipeline {
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
/*

    Mat myMat = new Mat();
    Mat disp = new Mat();

    int[] bestCol = new int[5];
    int count = 0;
    public boolean isProc = false;

    @Override
    public Mat processFrame(Mat input) {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        /*
        count++;
        input.copyTo(myMat);
        input.copyTo(disp);
        /*
        for(int y = input.cols() / 3; y < 2 * input.cols() / 3; y++) {
            for (int x = 0; x < input.rows(); x++) {
                if(x < input.rows() / 3f) {
                    double[] dispArr = disp.get(x, y);
                    totLight[0] += dispArr[0] + dispArr[1] + dispArr[2];
                }
                else if(x < 2 * input.rows() / 3f) {
                    double[] dispArr = disp.get(x, y);
                    totLight[1] += dispArr[0] + dispArr[1] + dispArr[2];
                }
                else {
                    double[] dispArr = disp.get(x, y);
                    totLight[2] += dispArr[0] + dispArr[1] + dispArr[2];
                }
            }
         */
        /*

        ArrayList<Point> edges = new ArrayList<>();
        for(int y = 0; y < input.cols() - 3; y++) {
            for (int x = 0; x < input.rows() - 3; x++) {
                double[] col = disp.get(x, y);
                double[] testcol = disp.get(x + 3, y);
                double dist = Math.sqrt(Math.pow(col[0] - testcol[0], 2) + Math.pow(col[1] - testcol[1], 2) + Math.pow(col[2] - testcol[2], 2));
                testcol = disp.get(x, y + 3);
                double secdist = Math.sqrt(Math.pow(col[0] - testcol[0], 2) + Math.pow(col[1] - testcol[1], 2) + Math.pow(col[2] - testcol[2], 2));
                if((dist > 75 || secdist > 75) && col[0] > 140 && col[1] > 100 && col[2] < 120) {
                    edges.add(new Point(x, y));
                }
            }
        }
        for(Point point : edges) {

        }



        if(!isProc && count == 5) {
            isProc = true;
        }

        Imgproc.rectangle(
                disp,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(bestCol[count % 5] == 0 ? 255 : 0, bestCol[count % 5] == 1 ? 255 : 0, bestCol[count % 5] == 2 ? 255 : 0), 4);

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        /*
        return disp;
    }

    public int getBestCol() {
        return (int)Math.round((bestCol[0] + bestCol[1] + bestCol[2] + bestCol[3] + bestCol[4]) / 4.0);
    }
}
*/