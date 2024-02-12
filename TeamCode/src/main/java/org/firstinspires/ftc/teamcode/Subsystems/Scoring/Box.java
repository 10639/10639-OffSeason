package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Box {

    private final HardwareMap hardwareMap;
    public DistanceSensor pixelDetector;
    public Box(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public enum boxInfo {
        EMPTY,
        ONE_PIXEL,
        FULL
    }

    private boxInfo Size;

    public void init() {
        pixelDetector = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Size = boxInfo.EMPTY;
    }

    public void loop(Telemetry telemetry) {
        double distance = pixelDetector.getDistance(DistanceUnit.CM);
        if(distance <= Constants.EMPTY_BOX_HIGH && distance > Constants.ONE_PIXEL_HIGH) {
           Size = boxInfo.EMPTY;
        }
        if(distance <= Constants.ONE_PIXEL_HIGH && distance > Constants.FULL_BOX_HIGH) {
            Size = boxInfo.ONE_PIXEL;
        }
        if(distance <= Constants.FULL_BOX_HIGH){
            Size = boxInfo.FULL;
        }
        switch(Size) {
            case EMPTY:
                telemetry.addLine("[Deposit Box]: Empty");
                break;
            case ONE_PIXEL:
                telemetry.addLine("[Deposit Box]: One Pixel Detected");
                break;
            case FULL:
                telemetry.addLine("[Deposit Box]: Two Pixels Detected");
                break;
        }
        telemetry.update();
    }



}