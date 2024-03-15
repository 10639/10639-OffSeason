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

@Autonomous(name = "👮‍♂️ Dev Auto", preselectTeleOp = "CenterStage_TeleOp")
public class TrajecTest extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Arm armSystem;
    public Intake intakeSystem;
    public Lift liftSystem;
    public Builder trajecBuilder;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        trajecBuilder = new Builder("BLUE", "LEFT");

        initializeSubsystems();

        trajecBuilder.calculatePoses(telemetry);
        trajecBuilder.trajDev = driveTrain.actionBuilder(trajecBuilder.initPose)
                .setReversed(true)
                .splineTo(trajecBuilder.centerVector, Math.toRadians(-90))
                .splineToLinearHeading(trajecBuilder.centerRetractPos, Math.toRadians(-90))
                .strafeTo(new Vector2d( (trajecBuilder.centerRetractPos.position.x) + 4 , trajecBuilder.centerRetractPos.position.y))
                .setReversed(true)
                .splineTo(trajecBuilder.centerScoringVector, Math.toRadians(0))
                .waitSeconds(1)
                .waitSeconds(1)
                .waitSeconds(1)
                .waitSeconds(1)
                .strafeToConstantHeading(trajecBuilder.parkingPose)
                .setReversed(false)
                .lineToX(trajecBuilder.backBoard_X + 15)
                .waitSeconds(0.5)
                .build();

        while (!isStarted()) {
            telemetry.addLine("Ready to Start! [Trajectory Testing)");
            telemetry.update();
        }
        waitForStart();

        //Built-in opModeIsActive check inside of runBlocking() so no need to double loop it by moving this inside of our opModeIsActive loop.
        Actions.runBlocking(new Helpers.RaceParallelCommand(
                trajecBuilder.trajDev,
                liftSystem.update(armSystem, telemetry)
        ));

    }
    private void initializeSubsystems() {
        liftSystem.init();
        armSystem.init();
        intakeSystem.init();
    }
}