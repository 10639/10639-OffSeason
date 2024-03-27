package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helpers.Controller;

public class Deposit {


    private final HardwareMap hardwareMap;
    public Lift liftSystem;
    public Arm armSystem;

    public Deposit(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {

        liftSystem = new Lift(hardwareMap);
        armSystem = new Arm(hardwareMap);

        liftSystem.init();
        armSystem.init();

    }

    public void loop(Controller Driver, Controller Operator, Telemetry telemetry) {

        liftSystem.loop(telemetry);
        liftSystem.driverControl(Driver);
        depositMacros(armSystem, Operator, telemetry);

    }

    private void depositMacros(Arm armSystem, Controller Operator, Telemetry telemetry) {
        double leftSlidePosition = liftSystem.leftSlide.getCurrentPosition();
        double slidePID = liftSystem.getPid();
        double slideTarget = liftSystem.getSlideTarget();
        boolean isScoreReady = (leftSlidePosition > 15) && (slideTarget != 0);

        armSystem.loop(Operator, isScoreReady, telemetry);
        Arm.ArmState armState = armSystem.getArmState();

        if(isScoreReady && (armState != Arm.ArmState.SCORING || slidePID < 0)) {
            armSystem.armIdle();
        } else if (slideTarget == 0) {
            zeroTargetBehavior(leftSlidePosition);
        }
    }

    public Action autoDeposit(Telemetry telemetry) {
        return t -> {
            liftSystem.loop(telemetry);
            double leftSlidePosition = liftSystem.leftSlide.getCurrentPosition();
            double slidePID = liftSystem.getPid();
            double slideTarget = liftSystem.getSlideTarget();
            boolean isScoreReady = (leftSlidePosition > 15) && (slideTarget != 0);
            Arm.ArmState armState = armSystem.getArmState();
            if(isScoreReady && (armState != Arm.ArmState.SCORING || slidePID < 0)) {
                armSystem.armIdle();
            } else if ((slideTarget== 0)) { //Properly De-Power Arm/Box
               zeroTargetBehavior(leftSlidePosition);
            }
            return true; // this returns true to make it loop forever; use RaceParallelCommand
        };
    }

    private void zeroTargetBehavior(double leftSlidePosition) {
        if(leftSlidePosition > 15) {
            armSystem.armIdle();
        } else if (leftSlidePosition < 7 && leftSlidePosition >= -1) {
            armSystem.dePower();
        }
    }

}
