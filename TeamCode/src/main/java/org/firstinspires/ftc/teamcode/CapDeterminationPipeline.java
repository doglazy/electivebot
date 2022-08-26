package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CapDeterminationPipeline extends OpenCvPipeline {
    enum CapPosition { //possibilities of cap position
        RIGHT,
        CENTER,
        LEFT
    }

    //color constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    //sets points
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(30, 0);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(255, 20);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(475, 25);
    static final int REGION_WIDTH = 100;
    static final int REGION_HEIGHT = 40;

    //creates points for rectangles
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);



    Mat region1_Cr, region2_Cr, region3_Cr;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    int avg1, avg2, avg3;

    private volatile CapPosition position = CapPosition.CENTER;

    void inputToCr(Mat input) { //extracts chroma red channel for analysis
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCr(firstFrame);

        //creates 3 boxes which will be the regions for detection (one for each possible cap position)
        region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
    }


    @Override
    public Mat processFrame(Mat input) {
        inputToCr(input);

        //average Cr values in each box
        avg1 = (int) Core.mean(region1_Cr).val[0];
        avg2 = (int) Core.mean(region2_Cr).val[0];
        avg3 = (int) Core.mean(region3_Cr).val[0];

        //outlines box area on camera stream
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                1); // Thickness of the rectangle lines


        //find min Cr value out of the 3 boxes -> the min will be where the orange part of cap is detected
        int min = Math.min(avg1, Math.min(avg2, avg3));

        //whichever is min, fill in box with green and assign position to according value (LEFT, CENTER, or RIGHT)
        if (min == avg1) {
            position = CapPosition.LEFT;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

        } else if (min == avg2) {
            position = CapPosition.CENTER;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

        } else if (min == avg3) {
            position = CapPosition.RIGHT;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        return input;
    }

    //to be called in auto files
    public boolean isCapLeft() {
        return position == CapPosition.LEFT;
    }
    public boolean isCapCenter() {
        return position == CapPosition.CENTER;
    }
    public boolean isCapRight() {
        return position == CapPosition.RIGHT;
    }

}
