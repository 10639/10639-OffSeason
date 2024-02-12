package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OffSeason_TeleOp")
public class TeleOp extends LinearOpMode {

    public MecanumDrive driveTrain;
    public PIDController controller;
    public DcMotorEx leftSlide, rightSlide;
    public Arm armSystem;
    public Intake intakeSystem;
    public Box pixelDetector;

    public static int target = 0;
    public static boolean rightSlideRest = true;
    public static boolean scoreAllowed = false;
    public static boolean tiltBox = false;

    public enum SpeedState {
        NORMAL(0.5),
        FAST(0.9);
        double multiplier = 0.5; //Default

        SpeedState(double value) {
            this.multiplier = value;
        }
    }

    SpeedState speedState;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new MecanumDrive(hardwareMap, Helpers.defaultTelePose);
        armSystem = new Arm(hardwareMap, telemetry);
        intakeSystem = new Intake(hardwareMap, telemetry);
        pixelDetector = new Box(hardwareMap, telemetry);

        controller = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);

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

        rightSlideRest = true;
        scoreAllowed = false;
        tiltBox = false;
        target = 0;
        speedState = SpeedState.NORMAL;


        while (!isStarted()) {
            telemetry.addLine("[driveTrain] Initialized");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.left_bumper) {
                speedState = SpeedState.NORMAL;
            } else if (gamepad1.right_bumper) {
                speedState = SpeedState.FAST;
            }

            driveTrain.initDrive(gamepad1, speedState.multiplier);
            armSystem.loop(gamepad2);
            intakeSystem.loop(gamepad2);

            if (gamepad1.square) {
                target = Constants.LIFT_FIRST_LEVEL;
            } else if (gamepad1.triangle) {
                target = Constants.LIFT_SECOND_LEVEL;
            } else if (gamepad1.circle) {
                target = Constants.LIFT_THIRD_LEVEL;
            } else if (gamepad1.cross) {
                armSystem.armIdle();
                target = Constants.LIFT_LEVEL_ZERO;
            }


            controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
            int leftPosition = leftSlide.getCurrentPosition();
            double pid = controller.calculate(leftPosition, target);
            double power = pid + Constants.Kf;
            if (pid < 0) { // Going down
                power = Math.max(power, -0.1);
                scoreAllowed = false;
            } else { //Going up
                power = Math.min(power, 0.8); //Power Range 0 -> 0.8;
            }
            leftSlide.setPower(power);
            rightSlide.setPower(power);
            if (leftSlide.getCurrentPosition() > 15) {
                rightSlideRest = false;
                scoreAllowed = true;
            }
            if (pid < 0) {
                armSystem.armIdle();
            }
            if (scoreAllowed) {
                if (gamepad2.cross || gamepad2.triangle) {
                    tiltBox = true;
                }
                if (tiltBox) {
                    armSystem.armScore();
                } else {
                    armSystem.armIdle();
                }

            }

            if ((target == 0)) { //Ensure Lifts are Fully Down (Observation: Right Slide Mainly Issues)
                armSystem.armIdle();
                scoreAllowed = false;
                tiltBox = false;
                if (leftSlide.getCurrentPosition() < 2 || (leftSlide.getCurrentPosition() < 0 && leftSlide.getCurrentPosition() >= -1)) {
                    armSystem.dePower();
                } else if (leftSlide.getCurrentPosition() > 2 || leftSlide.getCurrentPosition() < 0) {
                    while (leftSlide.getCurrentPosition() > 2 || leftSlide.getCurrentPosition() < 0) {
                        leftSlide.setPower((Math.signum(leftSlide.getCurrentPosition() * -1) * 0.3));
                    }
                    armSystem.dePower();
                }
            }

            switch(pixelDetector.getCount()) {
                case EMPTY:
                    telemetry.addLine("[Deposit Box] Empty");
                    break;
                case ONE_PIXEL:
                    telemetry.addLine("[Deposit Box] One Pixel Detected");
                    break;
                case FULL:
                    telemetry.addLine("[Deposit Box] Two Pixels Detected");
                    break;
            }
            telemetry.update();
        }
    }
}
