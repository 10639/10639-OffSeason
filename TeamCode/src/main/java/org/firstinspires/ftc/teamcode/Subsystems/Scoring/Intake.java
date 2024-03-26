package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Helpers.Constants;
import org.firstinspires.ftc.teamcode.Helpers.Controller;


public class Intake {

    private final HardwareMap hardwareMap;
    public DcMotorEx sweeper;
    public CRServo boxSweeper;
    public Servo intake;
    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        boxSweeper = hardwareMap.get(CRServo.class ,"boxSweeper");
        intake = hardwareMap.get(Servo.class, "intake");
        sweeper = hardwareMap.get(DcMotorEx.class, "sweeper");
        sweeper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        intake.scaleRange(0, 0.65);
        initIdle();
    }

    public void Sweep() { sweeper.setPower(Constants.Sweep); }
    public void reverseSweep() { sweeper.setPower(Constants.reverseSweep); }
    public void terminateSweep() { sweeper.setPower(Constants.terminatePower); }

    public void extendIntake() { intake.setPosition(Constants.intakeExtend); }

    public void retractIntake () { intake.setPosition(Constants.intakeRetract); }

    public void boxSweep() { boxSweeper.setPower(Constants.boxSweep); }

    public void boxReverseSweep() { boxSweeper.setPower(Constants.boxReverseSweep); }

    public void terminateBoxSweeper() { boxSweeper.setPower(Constants.terminatePower);}

    public void initIdle() {
        terminateSweep();
        terminateBoxSweeper();
        retractIntake();
    }

    public void loop(Controller Operator, Telemetry telemetry) {

      if(Operator.justPressed(Controller.Button.RIGHT_BUMPER)) { //Extend Intake + Spin Intake Pixels + Sweep inside Box
          extendIntake();
          Sweep();
          boxSweep();
      } else if(Operator.justPressed(Controller.Button.LEFT_BUMPER)) { //Reverse Intake Spin + Reverse outside Box + Retracts Intake
          retractIntake();
      } else if(Operator.justPressed(Controller.Button.DPAD_LEFT)) { //Reverses Box Spin
          boxReverseSweep();
      } else if(Operator.justPressed(Controller.Button.DPAD_DOWN)) { //Spin everything out
          reverseSweep();
          boxReverseSweep();
          retractIntake();
      } else if(Operator.justPressed(Controller.Button.DPAD_RIGHT)) {
          reverseSweep();
      }  else if(Operator.getLeftY() > 0 || Operator.getLeftY() < 0) { //Terminate Intake Spin
          terminateSweep();
          terminateBoxSweeper();
      }

        telemetry.addLine("--- Intake ---");
        telemetry.addData("Front Sweeper", sweeper.getPower() == -1 ? "Sweeping" : sweeper.getPower() == 1 ? "Reverse Sweeping" : "Terminated");
        telemetry.addData("Box Sweeper", boxSweeper.getPower() == -1 ? "Sweeping" : sweeper.getPower() == 1 ? "Reverse Sweeping" : "Terminated");
        telemetry.addData("Front Intake", intake.getPosition() == 1 ? "Extended" : "Retracted");
        telemetry.addData("Front Sweeper Current (Amps)", sweeper.getCurrent(CurrentUnit.AMPS));


    }

    //Actions for Autonomous

    public Action BoxIntake_SWEEPOUT() {
        return t -> {
            boxSweeper.setPower(0.5);
            return false;
        };
    }

    public Action BoxIntake_TERMINATE() {
        return t -> {
            boxSweeper.setPower(0);
            return false;
        };
    }





}

