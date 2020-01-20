package org.firstinspires.ftc.teamcode.ExperimentalCode.Util;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
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

    int[] bestCol = new int[10];
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
        count++;
        input.copyTo(myMat);
        input.copyTo(disp);
        Imgproc.GaussianBlur(myMat, disp, new Size(45, 45), 0);
        Rect roi = new Rect(new Point(0, input.cols() / 3f), new Point((input.rows()), 2 * input.cols() / 3f));

        int[] totLight = new int[3];

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


        for(int y = 0; y < input.cols(); y++) {
            for (int x = 0; x < input.rows(); x++) {
                if(!roi.contains(new Point(x, y))) {
                    disp.put(x, y, 0, 0, 0, 0);
                }
                else {
                    if(x < input.rows() / 3f) {
                        double[] dispArr = disp.get(x, y);
                        totLight[2] += dispArr[0] + dispArr[1];
                    }
                    else if(x < 2 * input.rows() / 3f) {
                        double[] dispArr = disp.get(x, y);
                        totLight[1] += dispArr[0] + dispArr[1];
                    }
                    else {
                        double[] dispArr = disp.get(x, y);
                        totLight[0] += dispArr[0] + dispArr[1];
                    }
                }
            }



            bestCol[count % 10] = totLight[0] < totLight[1] && totLight[0] < totLight[2] ? 0 : totLight[2] < totLight[0] && totLight[2] < totLight[1] ? 2 : 1;
            if(!isProc && count == 10) {
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
        }

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return disp;
    }

    public int getBestCol() {
        return (int)Math.round((bestCol[0] + bestCol[1] + bestCol[2] + bestCol[3] + bestCol[4] + bestCol[5] + bestCol[6] + bestCol[7] + bestCol[8] + bestCol[9]) / 10.0);
    }
}