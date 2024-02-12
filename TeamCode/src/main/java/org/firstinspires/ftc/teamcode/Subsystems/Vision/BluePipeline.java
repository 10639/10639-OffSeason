package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public BluePipeline(Telemetry t) {
        telemetry = t;
    } // constructor for the class to set up telemetry

    Mat mat = new Mat(); // declare a new matrix (computer representation of an image)
    Rect leftRect = new Rect(45, 75, 250, 250); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(500, 75, 350, 200);
    //    Rect rightRect = new Rect(1080, 230, 200, 117);
    final double PERCENT_THRESHOLD = Constants.CONFIDENCE; // define our threshold
    private int finalAnswer;
    Scalar blue = new Scalar(0, 0, 255); // define what the color of the rectangle outline is that appears on the output (blue)

    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT
    }

    private Location location;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)
        Scalar lowBlueBound = new Scalar(100, 40, 40); // set lower and upper bounds for the color we want to recognize (blue in this case)
        Scalar highBlueBound = new Scalar(125, 500, 500); //hue is out of 180 here, double it if searching it up online due to online websites going from 0-360

        Core.inRange(mat, lowBlueBound, highBlueBound, mat); // see which pixels are in our range, convert the pixels we're looking for into white, store it back to mat i think
        Mat left = mat.submat(leftRect); // create sub-matrices for our regions of interest
        Mat middle = mat.submat(midRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255; // get the percentage of white pixels that are present
        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]); // display the raw values to the driver hub
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);

        left.release();
        middle.release();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, leftRect, blue, 2); // draw the rectangles on the output matrix
        Imgproc.rectangle(mat, midRect, blue, 2);

        if(leftValue > PERCENT_THRESHOLD || midValue > PERCENT_THRESHOLD){
            if(leftValue > midValue) {
                location = Location.LEFT;
                telemetry.addData("Location", "Left");
                Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
            } else {
                location = Location.MIDDLE;
                telemetry.addData("Location", "Middle");
                Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);
            }
        } else {
            location = Location.RIGHT;
            telemetry.addData("Location", "Right");
            Imgproc.rectangle(mat, new Rect(100, 100, 50, 50), new Scalar(0, 255, 0), 2);
        }


        telemetry.update();
        return mat;
    }

    public Location getLocation() {
        return location;
    }

    public static void stop(OpenCvCamera camera) {
        camera.stopStreaming();
    }
}