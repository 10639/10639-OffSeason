package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Helpers.Constants;

public class Lift {

    private final HardwareMap hardwareMap;
    public DcMotorEx leftSlide, rightSlide;
    private PIDController controller;
    private double pid;
    public int target;

    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.target = 0;
    }

    public void init() {
        controller = new PIDController(Constants.SlidesPIDF.Kp.getValue(), Constants.SlidesPIDF.Ki.getValue(), Constants.SlidesPIDF.Kd.getValue());
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
        target = 0;
    }

    public void loop(Telemetry telemetry) {
        controller.setPID(Constants.SlidesPIDF.Kp.getValue(), Constants.SlidesPIDF.Ki.getValue(), Constants.SlidesPIDF.Kd.getValue());
        int leftPosition = leftSlide.getCurrentPosition();
        pid = controller.calculate(leftPosition, target);
        double power = pid + Constants.SlidesPIDF.Kf.getValue();
        if (pid < 0) { // Going down
            power = Math.max(power, Constants.VelocityConfig.LIFT_TERMINAL.getVelocity());
        } else { //Going up
            power = Math.min(power, Constants.VelocityConfig.LIFT_UP.getVelocity());
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addLine("--- Slides ---");
        telemetry.addData("Slides Target", target);
        telemetry.addData("Left Slide Position", leftPosition);
        telemetry.addData("Right Slide Position", rightSlide.getCurrentPosition());
        telemetry.addData("Left Slide Current (Amps)", leftSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Slide Current (Amps)", rightSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Power Allocated", power);

    }

    //Methods

    public void setSlideTarget(int newTarget) {
        target = newTarget;
    }

    public double getPid() {
        return pid;
    }

    public int getSlideTarget() {
        return target;
    }


    //Actions for Autonomous

    public Action setSlideAction(int newTarget) {
        return t -> {
            target = newTarget;
            return false;
        };
    }
    public Action update(Arm armSystem, Telemetry telemetry) {
        return t -> {
            loop(telemetry);
            double leftSlidePosition = leftSlide.getCurrentPosition();
            Arm.ArmState armState = armSystem.getArmState();
            if (leftSlidePosition > 15) {
                if( ((armState != Arm.ArmState.SCORING) || getPid() < 0)) {
                    armSystem.armIdle();
                }
            }

            if ((getSlideTarget() == 0)) { //Properly De-Power Arm/Box
                if(leftSlidePosition > 15) {
                    armSystem.armIdle();
                } else if (leftSlidePosition < 2 && leftSlidePosition >= -1) {
                    armSystem.dePower();
                }
            }
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }




}
