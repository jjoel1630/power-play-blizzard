package org.firstinspires.ftc.teamcode.drive.opmode.imagedetect;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class SleeveColorDetection extends OpenCvPipeline {
    // 0 for left, 1 for center, 2 for right
    private int position = 1;

    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(150, 80);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 40;

    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    @Override
    public Mat processFrame(Mat input) {
        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        if (sumColors.val[0] == minColor) {
            position = 1;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (sumColors.val[1] == minColor) {
            position = 2;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        } else {
            position = 0;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        }

        areaMat.release();
        return input;
    }

    public int getParkPosition() {
        return position;
    }
}
