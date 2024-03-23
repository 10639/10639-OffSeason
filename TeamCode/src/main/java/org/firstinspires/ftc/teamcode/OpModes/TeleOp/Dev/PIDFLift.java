package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDFLift_Test")
public class PIDFLift extends OpMode {

    private PIDController controller;
    public DcMotorEx leftSlide, rightSlide;
    public Arm armSystem;
    public static int target = 0;
    public static double p = 0.1, i = 0, d = 0.00001;
    //Possible Value for P: 0.1;
    //Possible Value for D: 0.00001;
    public static double f = 0.12; //Possible Value 0.12;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        armSystem = new Arm(hardwareMap);
        armSystem.init();

        rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {

        controller.setPID(Constants.SlidesPIDF.Kp.getValue(), Constants.SlidesPIDF.Ki.getValue(), Constants.SlidesPIDF.Kd.getValue());
        int leftPosition = leftSlide.getCurrentPosition();
        double pid = controller.calculate(leftPosition, target);
        double power = pid + Constants.SlidesPIDF.Kf.getValue();
        if (pid < 0) { // Going down
            power = Math.max(power, -0.1);
        } else { //Going up
            power = Math.min(power, 1.0); //Power Range 0 -> 1;
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        double leftSlidePosition = leftSlide.getCurrentPosition();
        if (leftSlidePosition > 15) {
            if( (!(Arm.AUTON_SCORING) || pid < 0)) {
                armSystem.armIdle();
            }
        }

        if (target == 0) { //Properly De-Power Arm/Box
            if(leftSlidePosition > 15) {
                armSystem.armIdle();
            } else if (leftSlidePosition < 2 && leftSlidePosition >= -1) {
                armSystem.dePower();
            }
        }

        telemetry.addData("leftPos", leftPosition);
        telemetry.addData("rightPos", rightSlide.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("Calculated PID", pid);
        telemetry.addData("Slides Power", power);
        telemetry.addData("Slide Direction:", pid < 0 ? "Down" : "Up");
        telemetry.update();

    }
}
