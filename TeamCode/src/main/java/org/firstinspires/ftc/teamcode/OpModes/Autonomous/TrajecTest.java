package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;

@Autonomous(name = "👮‍♂️ Dev Auto", preselectTeleOp = "CenterStage_TeleOp")
public class TrajecTest extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Arm armSystem;
    public Intake intakeSystem;
    public Lift liftSystem;
    public TrajectoryBuilder trajecBuilder;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        trajecBuilder = new TrajectoryBuilder("BLUE", "LEFT");

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