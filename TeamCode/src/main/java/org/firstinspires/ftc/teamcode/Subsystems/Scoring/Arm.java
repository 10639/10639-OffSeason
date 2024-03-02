package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private final HardwareMap hardwareMap;
    public ServoImplEx leftPivot, rightPivot;
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public static boolean AUTON_SCORING; //Used to check if the box is currently in a scoring mode for macro verification in auto

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
    }

    public void armScore() {
        rightPivot.scaleRange(0, 0.675);

        leftPivot.setPwmDisable();
        rightPivot.setPosition(1);
    }

    public void dePower() {
        leftPivot.setPwmDisable();
        rightPivot.setPwmDisable();
        AUTON_SCORING = false;
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        //Empty for now
    }

    public Action Arm_IDLE() {
        return t -> {
            armIdle();
            AUTON_SCORING = false;
            return false;
        };
    }

    public Action Arm_SCORE() {
        return t -> {
            armScore();
            AUTON_SCORING = true;
            return false;
        };
    }

    public Action Arm_DEPOWER() {
        return t -> {
            dePower();
            AUTON_SCORING = false;
            return false;
        };
    }
}