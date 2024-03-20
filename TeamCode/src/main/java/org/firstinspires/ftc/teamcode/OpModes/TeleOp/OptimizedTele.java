package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Cache;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;

@TeleOp(name = "Optimized_Tele")
public class OptimizedTele extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Cache bulkCache;
    public Lift liftSystem;
    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;
    public ElapsedTime loopTime;

    public static boolean scoreAllowed = false;
    public static boolean tiltBox = false;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        bulkCache = new Cache(hardwareMap);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        pixelDetector = new Box(hardwareMap);
        loopTime = new ElapsedTime();

        initializeSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            loopSubsystems();
            handleScoringConditions();
            telemetry.update();
        }
    }

    private void handleScoringConditions() {
        int target = liftSystem.getSlideTarget();
        double leftSlidePosition = liftSystem.leftSlide.getCurrentPosition();

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

    private void initializeSubsystems() {
        tiltBox = false;
        scoreAllowed = false;

        bulkCache.init();
        liftSystem.init();
        armSystem.init();
        intakeSystem.init();
        pixelDetector.init();

        telemetry.addLine("--- Scoring Conditions Initialized ---");
        telemetry.addLine("--- Subsystems Initialized ---");
        telemetry.update();
    }

    private void loopSubsystems() {
        bulkCache.resetBulkCache(telemetry);
        loopTime.reset();

        driveTrain.loop(gamepad1, telemetry);
        armSystem.loop(gamepad2, telemetry);
        intakeSystem.loop(gamepad2, telemetry);
        pixelDetector.loop(telemetry);
        updateLiftTargets(gamepad1);
        liftSystem.loop(telemetry);

        double loopTimeMs = loopTime.milliseconds();
        telemetry.addLine("--- Loop Times ---");
        telemetry.addData("loopTimeMs", loopTimeMs);
        telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
    }

    private void updateLiftTargets(Gamepad gamepad) {
        if (gamepad.square) {
            liftSystem.setSlideTarget(Constants.LIFT_FIRST_LEVEL);
        } else if (gamepad.triangle) {
            liftSystem.setSlideTarget(Constants.LIFT_SECOND_LEVEL);
        } else if (gamepad.circle) {
            liftSystem.setSlideTarget(Constants.LIFT_THIRD_LEVEL);
        } else if (gamepad.cross) {
            liftSystem.setSlideTarget(Constants.LIFT_LEVEL_ZERO);
        }
    }


}


