package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Controller;

public class Arm {

    private final HardwareMap hardwareMap;
    private Telemetry telemetry;
    public ServoImplEx leftPivot, rightPivot;
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
    public enum ArmState { SCORING, IDLE, DEPOWERED }
    public ArmState State;

    public void init() {
         leftPivot = hardwareMap.get(ServoImplEx.class, "leftPivot");
         rightPivot = hardwareMap.get(ServoImplEx.class, "rightPivot");
         dePower();
    }

    public void armIdle() {
        leftPivot.scaleRange(0, 0.1); //Decreasing (max) => Lifts Box Higher
        rightPivot.scaleRange(0, 0.1); //^

        leftPivot.setPosition(0);
        rightPivot.setPosition(1);
        State = ArmState.IDLE;
    }

    public void armScore() {
        rightPivot.scaleRange(0, 0.675);

        leftPivot.setPwmDisable();
        rightPivot.setPosition(1);
        State = ArmState.SCORING;
    }

    public void dePower() {
        leftPivot.setPwmDisable();
        rightPivot.setPwmDisable();
        State = ArmState.DEPOWERED;
    }

    public void loop(Controller Operator, Boolean isScoreReady) {

        if ((Operator.justPressed(Controller.Button.CROSS) || Operator.justPressed(Controller.Button.TRIANGLE)) && isScoreReady) {
            armScore();
        }
        telemetry.addLine("--- Arm ---");
        telemetry.addData("Arm State", getArmState());

    }

    public ArmState getArmState(){
        return State;
    }

    public Action Arm_IDLE() {
        return t -> {
            armIdle();
            return false;
        };
    }

    public Action Arm_SCORE() {
        return t -> {
            armScore();
            return false;
        };
    }

    public Action Arm_DEPOWER() {
        return t -> {
            dePower();
            return false;
        };
    }
}