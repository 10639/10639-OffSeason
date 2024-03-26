package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Helpers.Controller;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;

@TeleOp(name = "👑 OffSeason TeleOp ")
public class Main extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Lift liftSystem;
    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;
    public ElapsedTime loopTime;
    public Controller Driver;
    public Controller Operator;

    public static boolean isTiltBoxEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);
        intakeSystem = new Intake(hardwareMap);
        pixelDetector = new Box(hardwareMap);
        loopTime = new ElapsedTime();
        Driver = new Controller(gamepad1);
        Operator = new Controller(gamepad2);

        initializeSubsystems();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            loopSubsystems();
        }
    }

    private void handleScoringConditions() {
        int target = liftSystem.getSlideTarget();
        double leftSlidePosition = liftSystem.leftSlide.getCurrentPosition();
        double slidePID = liftSystem.getPid();
        boolean isScoreReady = (leftSlidePosition > 15) && (target != 0);

        if(isScoreReady && (!isTiltBoxEnabled || slidePID < 0)) {
            armSystem.armIdle();
        }

        if ((Operator.justPressed(Controller.Button.CROSS) || Operator.justPressed(Controller.Button.TRIANGLE)) && isScoreReady) {
            isTiltBoxEnabled = true;
            armSystem.armScore();
        }

        if (target == 0) {
            isTiltBoxEnabled = false;
            if(leftSlidePosition > 15) {
                armSystem.armIdle();
            } else if (leftSlidePosition < 7 && leftSlidePosition >= -1) {
                armSystem.dePower();
            }
        }
    }

    private void initializeSubsystems() {
        isTiltBoxEnabled = false;

        liftSystem.init();
        armSystem.init();
        intakeSystem.init();
        pixelDetector.init();

        telemetry.addLine("--- Scoring Conditions Initialized ---");
        telemetry.addLine("--- Subsystems Initialized ---");
        telemetry.update();
    }

    private void loopSubsystems() {
        loopTime.reset();
        Operator.readButtons();
        Driver.readButtons();

        driveTrain.loop(Driver, telemetry);
        armSystem.loop(Operator, telemetry);
        intakeSystem.loop(Operator, telemetry);
        pixelDetector.loop(telemetry);

        liftSystem.loop(telemetry);
        updateLiftTargets(Driver);
        handleScoringConditions();

        double loopTimeMs = loopTime.milliseconds();
        telemetry.addLine("--- Loop Times ---");
        telemetry.addData("loopTimeMs", loopTimeMs);
        telemetry.addData("loopTimeHz", 1000.0 / loopTimeMs);
        telemetry.update();
    }

    private void updateLiftTargets(Controller Driver) {
        if (Driver.justPressed(Controller.Button.SQUARE)) {
            liftSystem.setSlideTarget(Constants.SlidePositions.LOW.getTicks());
        } else if (Driver.justPressed(Controller.Button.TRIANGLE)) {
            liftSystem.setSlideTarget(Constants.SlidePositions.MEDIUM.getTicks());
        } else if (Driver.justPressed(Controller.Button.CIRCLE)) {
            liftSystem.setSlideTarget(Constants.SlidePositions.HIGH.getTicks());
        } else if (Driver.justPressed(Controller.Button.CROSS)) {
            liftSystem.setSlideTarget(Constants.SlidePositions.DOWN.getTicks());
        }
    }
}
