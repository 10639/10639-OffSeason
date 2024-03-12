package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Dev;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Optimized_Tele")
public class OptimizedTele extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Lift liftSystem;
    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;

    public static int target = 0;
    public static double leftSlidePosition = 0;
    public static boolean scoreAllowed = false;
    public static boolean tiltBox = false;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        pixelDetector = new Box(hardwareMap);
        initializeSubsystems();

        tiltBox = false;
        scoreAllowed = false;
        leftSlidePosition = 0;

        telemetry.addLine("--- DriveTrain Initialized ---");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.loop(gamepad1, telemetry);
            armSystem.loop(gamepad2, telemetry);
            intakeSystem.loop(gamepad2, telemetry);
            pixelDetector.loop(telemetry);
            updateLiftTargets(gamepad1);
            liftSystem.loop(telemetry);

            handleScoringConditions();
            logTelemetry();
        }
    }

    private void initializeSubsystems() {
        liftSystem.init();
        armSystem.init();
        intakeSystem.init();
        pixelDetector.init();
    }

    private void updateLiftTargets(Gamepad gamepad) {
        if (gamepad.square) {
            liftSystem.updateTarget(Constants.LIFT_FIRST_LEVEL);
        } else if (gamepad.triangle) {
            liftSystem.updateTarget(Constants.LIFT_SECOND_LEVEL);
        } else if (gamepad.circle) {
            liftSystem.updateTarget(Constants.LIFT_THIRD_LEVEL);
        } else if (gamepad.cross) {
            liftSystem.updateTarget(Constants.LIFT_LEVEL_ZERO);
        }
    }

    private void handleScoringConditions() {
        target = liftSystem.getTarget();
        leftSlidePosition = liftSystem.leftSlide.getCurrentPosition();

        if (leftSlidePosition > 15) {
            scoreAllowed = true;
            if (!tiltBox || liftSystem.getPid() < 0) {
                armSystem.armIdle();
            }
        }

        if ((gamepad2.cross || gamepad2.triangle) && scoreAllowed) {
            tiltBox = true;
            armSystem.armScore();
        }

        if (target == 0) {
            scoreAllowed = false;
            tiltBox = false;
            if(leftSlidePosition > 15) {
                armSystem.armIdle();
            } else if (leftSlidePosition < 2 && leftSlidePosition >= -1) {
                armSystem.dePower();
            }
        }
    }

    private void logTelemetry() {
        telemetry.addLine("--- Motor Voltages ---");
        telemetry.addData("Intake Current (Amps)", intakeSystem.sweeper.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Left Slide Current (Amps)", liftSystem.leftSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Slide Current (Amps)", liftSystem.rightSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }

}


