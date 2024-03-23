package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class Pipeline extends OpenCvPipeline {

    Mat mat = new Mat();

    String alliance;
    Scalar lowHSV;
    Scalar highHSV;
    Rect LEFT_RECT, CENTER_RECT, RIGHT_RECT;

    double leftRectValue, centerRectValue, rightRectValue;
    double PERCENT_THRESHOLD = Constants.CONFIDENCE; // define our threshold

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    public enum Location { LEFT, CENTER, RIGHT }
    private Location location;

    public Pipeline(String alliance) {
        this.alliance = alliance;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)

        switch(alliance) {
            case "BLUE":
                lowHSV = new Scalar(100, 40, 40);
                highHSV = new Scalar(125, 500, 500);
                break;
            case "RED":
                lowHSV = new Scalar(0, 60, 60);
                highHSV = new Scalar(20, 500, 500);
                break;
        }

        LEFT_RECT = new Rect(45, 75, 250, 250);
        CENTER_RECT = new Rect(500, 75, 250, 250);
        RIGHT_RECT = new Rect(700, 75, 250, 250);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat Left = mat.submat(RIGHT_RECT);
        Mat Center = mat.submat(CENTER_RECT);
        Mat Right = mat.submat(LEFT_RECT);

        leftRectValue = Core.sumElems(Left).val[0] / LEFT_RECT.area() / 255;
        centerRectValue = Core.sumElems(Center).val[0] / CENTER_RECT.area() / 255;
        rightRectValue = Core.sumElems(Right).val[0] / RIGHT_RECT.area() / 255;


        Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, CENTER_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 5);

        if (leftRectValue > PERCENT_THRESHOLD && leftRectValue > centerRectValue && leftRectValue > rightRectValue) {
            Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 10);
            location = Location.LEFT;
        } else if (rightRectValue > PERCENT_THRESHOLD && rightRectValue > centerRectValue && rightRectValue > leftRectValue) {
            Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 10);
            location = Location.RIGHT;
        } else {
            Imgproc.rectangle(mat, CENTER_RECT, new Scalar(60, 255, 255), 10);
            location = Location.CENTER;
        }

        telemetry.addData("Left Region %", leftRectValue);
        telemetry.addData("Center Region %", centerRectValue);
        telemetry.addData("Right Region %", rightRectValue);
        telemetry.addData("Determined Region", location);
        telemetry.update();

        Left.release();
        Center.release();
        Right.release();

        return mat;
    }

    public Location getLocation(){
        return location;
    }
}