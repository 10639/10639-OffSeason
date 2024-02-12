package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Dev;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.RedPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "VisionTest_Red", preselectTeleOp = "CenterStage_TeleOp")
public class VisionTest_Red extends LinearOpMode {

    public SampleMecanumDrive driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new SampleMecanumDrive(hardwareMap);
        RedPipeline pipeline;
        OpenCvWebcam webcam;

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory
                .getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, "Webcam"),
                        cameraMonitorViewId);

        pipeline = new RedPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        RedPipeline.Location location = null;
        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Location", location);
            telemetry.update();
        }

    }
}