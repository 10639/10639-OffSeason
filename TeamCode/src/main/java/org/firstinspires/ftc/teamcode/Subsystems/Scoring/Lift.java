package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Helpers.Constants;

public class Lift {

    private final HardwareMap hardwareMap;
    public DcMotorEx leftSlide, rightSlide;
    public PIDController controller;
    private double pid;
    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
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
    }

    public void loop(int target, Telemetry telemetry) {
        controller.setPID(Constants.Kp, Constants.Ki, Constants.Kd);
        int leftPosition = leftSlide.getCurrentPosition();
        pid = controller.calculate(leftPosition, target);
        double power = pid + Constants.Kf;
        if (pid < 0) { // Going down
            power = Math.max(power, Constants.MAX_DOWN_VELO);
        } else { //Going up
            power = Math.min(power, Constants.MAX_UP_VELO); //Power Range 0 -> 0.8;
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addLine("--- Slides ---");
        telemetry.addData("Slides Target", target);
        telemetry.addData("Left Slide Position", leftPosition);
        telemetry.addData("Right Slide Position", rightSlide.getCurrentPosition());
        telemetry.addData("Power Allocated", power);

    }
    public double getPid() {
        return pid;
    }
}