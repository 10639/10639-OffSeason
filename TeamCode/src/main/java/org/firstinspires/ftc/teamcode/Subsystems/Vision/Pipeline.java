package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class Pipeline extends OpenCvPipeline {

    Telemetry telemetry;
    public Pipeline(Telemetry t) {

        telemetry = t;

    }
    Mat mat = new Mat();
    Mat mat1 = new Mat();
    Mat mat2 = new Mat();

    Rect leftRect = new Rect(45, 75, 250, 250); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(500, 75, 350, 200);

    double PERCENT_THRESHOLD = Constants.CONFIDENCE; // define our threshold
    private int finalAnswer;
    Scalar red = new Scalar(255, 0, 0); // define what the color of the rectangle outline is that appears on the output (red)

    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT
    }

    private Location location;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)
        Scalar lowRedBound = new Scalar(0, 60, 60); // set lower and upper bounds for the color we want to recognize (red in this case)
        Scalar highRedBound = new Scalar(20, 500, 500);

        Scalar lowRedBound2 = new Scalar(160, 60, 60);
        Scalar highRedBound2 = new Scalar(180, 255, 255);

        Core.inRange(mat, lowRedBound, highRedBound, mat1);
        Core.inRange(mat, lowRedBound2, highRedBound2, mat2);
        Core.bitwise_or(mat1, mat2, mat);

        Mat middle = mat.submat(midRect);
        Mat left = mat.submat(leftRect);

        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;
        double rightValue = Core.sumElems(left).val[0] / leftRect.area() / 255;

        telemetry.addData("Middle percentage value", midValue);
        telemetry.addData("Right percentage value", rightValue);

        left.release();
        middle.release();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, leftRect, red, 2);
        Imgproc.rectangle(mat, midRect, red, 2);

        if (rightValue > PERCENT_THRESHOLD || midValue > PERCENT_THRESHOLD){
            if(rightValue > midValue) {
                location = Pipeline.Location.RIGHT;
                telemetry.addData("Location", "Left");
                Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
            } else {
                location = Pipeline.Location.MIDDLE;
                telemetry.addData("Location", "Middle");
                Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);
            }
        } else {
            location = Location.RIGHT;
            telemetry.addData("Location", "Right");
            Imgproc.rectangle(mat, new Rect(100, 100, 50, 50), new Scalar(0, 255, 0), 2);
        }
        //TODO: Right Default
        return mat;
    }

    public Location getLocation(){

        return location;

    }


}
