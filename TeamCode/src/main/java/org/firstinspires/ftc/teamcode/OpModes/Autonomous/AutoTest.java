package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Trajectories.Builder;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Pipeline;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Webcam;

@Autonomous(name = "🟦 Left Backdrop [WIP]", preselectTeleOp = "CenterStage_TeleOp")
public class AutoTest extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Arm armSystem;
    public Intake intakeSystem;
    public Lift liftSystem;
    public Box pixelDetector;
    public Webcam Camera;
    public Builder trajecBuilder;
    public Pipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        pixelDetector = new Box(hardwareMap);
        Camera = new Webcam(hardwareMap);
        pipeline = new Pipeline(telemetry);
        trajecBuilder = new Builder("BLUE", "LEFT");

        initializeSubsystems();
        Pipeline.Location location = Pipeline.Location.RIGHT;
        Camera.device.setPipeline(pipeline);

        trajecBuilder.calculatePoses(telemetry);
        trajecBuilder.trajLeft = driveTrain.actionBuilder(trajecBuilder.initPose)
                .setReversed(true)
                .splineTo(trajecBuilder.midwayVector, Math.toRadians(-90))
                .strafeToConstantHeading(trajecBuilder.leftVector)
                .splineToLinearHeading(trajecBuilder.leftRetractPos, Math.toRadians(-90))
                .setReversed(true)
                .splineTo(trajecBuilder.leftScoringVector, Math.toRadians(0))
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        liftSystem.setSlideAction(Constants.LIFT_FIRST_LEVEL),
                        armSystem.Arm_IDLE()
                ))
                .waitSeconds(1)
                .afterTime(0, armSystem.Arm_SCORE())
                .waitSeconds(1)
                .afterTime(0, intakeSystem.BoxIntake_SWEEPOUT())
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        intakeSystem.BoxIntake_TERMINATE(),
                        liftSystem.setSlideAction(Constants.LIFT_LEVEL_ZERO),
                        armSystem.Arm_IDLE()
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
                        liftSystem.setSlideAction(Constants.LIFT_FIRST_LEVEL),
                        armSystem.Arm_IDLE()
                ))
                .waitSeconds(1)
                .afterTime(0, armSystem.Arm_SCORE())
                .waitSeconds(1)
                .afterTime(0, intakeSystem.BoxIntake_SWEEPOUT())
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        intakeSystem.BoxIntake_TERMINATE(),
                        liftSystem.setSlideAction(Constants.LIFT_LEVEL_ZERO),
                        armSystem.Arm_IDLE()
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
                        liftSystem.setSlideAction(Constants.LIFT_FIRST_LEVEL),
                        armSystem.Arm_IDLE()
                ))
                .waitSeconds(1)
                .afterTime(0, armSystem.Arm_SCORE())
                .waitSeconds(1)
                .afterTime(0, intakeSystem.BoxIntake_SWEEPOUT())
                .waitSeconds(1)
                .afterTime(0, new SequentialAction(
                        intakeSystem.BoxIntake_TERMINATE(),
                        liftSystem.setSlideAction(Constants.LIFT_LEVEL_ZERO),
                        armSystem.Arm_IDLE()
                ))
                .strafeToConstantHeading(trajecBuilder.parkingPose)
                .setReversed(false)
                .lineToX(trajecBuilder.backBoard_X + 15)
                .waitSeconds(0.5)
                .build();


        while (!isStarted()) {
            location = pipeline.getLocation();
            telemetry.addData("Identified Location", location);
            telemetry.addLine("Ready to Start! [Preload + Park)");
            telemetry.update();
        }
        waitForStart();

        //Built-in opModeIsActive check inside of runBlocking() so no need to double loop it by moving this inside of our opModeIsActive loop.
        Actions.runBlocking(new Helpers.RaceParallelCommand(
                location == Pipeline.Location.MIDDLE ?
                        trajecBuilder.trajCenter :
                        location == Pipeline.Location.LEFT
                                ? trajecBuilder.trajLeft
                                : trajecBuilder.trajRight,
                liftSystem.update(armSystem, telemetry)
        ));
        Camera.stopStream();

    }

    private void initializeSubsystems() {
        liftSystem.init();
        armSystem.init();
        intakeSystem.init();
        pixelDetector.init();
        Camera.init();
    }
}