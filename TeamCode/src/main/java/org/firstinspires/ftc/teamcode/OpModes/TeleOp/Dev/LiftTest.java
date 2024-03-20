package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Helpers;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Lift;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ManualLift")
public class LiftTest extends LinearOpMode {

    public MecanumDrive driveTrain;
    public Lift liftSystem;
    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;
    public DcMotorEx leftSlide, rightSlide;

    public static int power = 0;
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

        telemetry.addLine("--- Subsystems Initialized ---");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.loop(gamepad1, telemetry);
            armSystem.loop(gamepad2, telemetry);
            intakeSystem.loop(gamepad2, telemetry);
            pixelDetector.loop(telemetry);
            handleScoringConditions();
            logTelemetry();
        }
    }

    private void initializeSubsystems() {
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");

        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        armSystem.init();
        intakeSystem.init();
        pixelDetector.init();
    }


    private void handleScoringConditions() {
        leftSlidePosition = liftSystem.leftSlide.getCurrentPosition();
        power = (int) -gamepad2.right_stick_y;
        leftSlide.setPower(power);
        rightSlide.setPower(power);

        if (leftSlidePosition > 15) {
            scoreAllowed = true;
            if (!tiltBox || power < 0) {
                armSystem.armIdle();
            }
        }

        if ((gamepad2.cross || gamepad2.triangle) && scoreAllowed) {
            tiltBox = true;
            armSystem.armScore();
        }

        if (power < 0) {
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


