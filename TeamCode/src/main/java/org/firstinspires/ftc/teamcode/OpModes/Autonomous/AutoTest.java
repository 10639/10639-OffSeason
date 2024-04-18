package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Helpers.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Pipeline;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Webcam;

@Autonomous(name = "🟦 Left Backdrop [WIP]", preselectTeleOp = "CenterStage_TeleOp")
public class AutoTest extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Deposit depositSystem;
    public Intake intakeSystem;
    public Box pixelDetector;
    public Webcam Camera;
    public TrajectoryBuilder trajecBuilder;
    public Pipeline pipeline;
    Pipeline.Location location;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        depositSystem = new Deposit(hardwareMap, telemetry);
        intakeSystem = new Intake(hardwareMap, telemetry);
        pixelDetector = new Box(hardwareMap, telemetry);
        Camera = new Webcam(hardwareMap);
        pipeline = new Pipeline("BLUE");
        trajecBuilder = new TrajectoryBuilder("BLUE", "LEFT", telemetry);

        initializeSubsystems();
        setTrajectoryActions();


        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Identified Location", location);
            telemetry.addLine("Ready to Start! [Preload + Park)");
            telemetry.update();
        }

        waitForStart();

        //Built-in opModeIsActive check inside of runBlocking() so no need to double loop it by moving this inside of our opModeIsActive loop.
        Actions.runBlocking(new Helpers.RaceParallelCommand(
                location == Pipeline.Location.CENTER ?
                        trajecBuilder.trajCenter :
                        location == Pipeline.Location.LEFT
                                ? trajecBuilder.trajLeft
                                : trajecBuilder.trajRight,
                depositSystem.autoDeposit()
        ));
        Camera.stopStream();

    }

    private void setTrajectoryActions() {
        trajecBuilder.trajLeft = driveTrain.actionBuilder(trajecBuilder.initPose)
                .setReversed(true)
                .splineTo(trajecBuilder.midwayVector, Math.toRadians(-90))
                .strafeToConstantHeading(trajecBuilder.leftVector)
                .splineToLinearHeading(trajecBuilder.leftRetractPos, Math.toRadians(-90))
                .setReversed(true)
                .splineTo(trajecBuilder.leftScoringVector, Math.toRadians(0))
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        depositSystem.liftSystem.setSlideAction(Constants.SlidePositions.LOW.getTicks())
                ))
                .waitSeconds(1)
                .afterTime(0, depositSystem.armSystem.Arm_SCORE())
                .waitSeconds(1)
                .afterTime(0, intakeSystem.BoxIntake_SWEEPOUT())
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        intakeSystem.BoxIntake_TERMINATE(),
                        depositSystem.liftSystem.setSlideAction(Constants.SlidePositions.DOWN.getTicks())
                ))
                .strafeToConstantHeading(trajecBuilder.parkingPose)
                .setReversed(false)
                .lineToX(trajecBuilder.backBoard_X + 15)
                .waitSeconds(0.5)
                .build();


        trajecBuilder.trajCenter = driveTrain.actionBuilder(trajecBuilder.initPose)
                .setReversed(true)
                .splineTo(trajecBuilder.centerVector, Math.toRadians(-90))
                .splineToLinearHeading(trajecBuilder.centerRetractPos, Math.toRadians(-90))
                .strafeTo(new Vector2d( (trajecBuilder.centerRetractPos.position.x) + 4 , trajecBuilder.centerRetractPos.position.y))
                .setReversed(true)
                .splineTo(trajecBuilder.centerScoringVector, Math.toRadians(0))
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        depositSystem.liftSystem.setSlideAction(Constants.SlidePositions.LOW.getTicks())
                ))
                .waitSeconds(1)
                .afterTime(0, depositSystem.armSystem.Arm_SCORE())
                .waitSeconds(1)
                .afterTime(0, intakeSystem.BoxIntake_SWEEPOUT())
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        intakeSystem.BoxIntake_TERMINATE(),
                        depositSystem.liftSystem.setSlideAction(Constants.SlidePositions.DOWN.getTicks())
                ))
                .strafeToConstantHeading(trajecBuilder.parkingPose)
                .setReversed(false)
                .lineToX(trajecBuilder.backBoard_X + 15)
                .waitSeconds(0.5)
                .build();

        trajecBuilder.trajRight = driveTrain.actionBuilder(trajecBuilder.initPose)
                .setReversed(true)
                .splineTo(trajecBuilder.midwayVector, Math.toRadians(-90))
                .strafeToConstantHeading(trajecBuilder.rightVector)
                .splineToLinearHeading(trajecBuilder.rightRetractPos, Math.toRadians(-90))
                .setReversed(true)
                .splineTo(trajecBuilder.rightScoringVector, Math.toRadians(0))
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        depositSystem.liftSystem.setSlideAction(Constants.SlidePositions.LOW.getTicks())
                ))
                .waitSeconds(1)
                .afterTime(0, depositSystem.armSystem.Arm_SCORE())
                .waitSeconds(1)
                .afterTime(0, intakeSystem.BoxIntake_SWEEPOUT())
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        intakeSystem.BoxIntake_TERMINATE(),
                        depositSystem.liftSystem.setSlideAction(Constants.SlidePositions.DOWN.getTicks())
                ))
                .strafeToConstantHeading(trajecBuilder.parkingPose)
                .setReversed(false)
                .lineToX(trajecBuilder.backBoard_X + 15)
                .waitSeconds(0.5)
                .build();
    }

    private void initializeSubsystems() {
        Camera.init();
        depositSystem.init();
        intakeSystem.init();
        pixelDetector.init();

        location = Pipeline.Location.RIGHT;
        Camera.device.setPipeline(pipeline);
        trajecBuilder.calculate();
    }

}