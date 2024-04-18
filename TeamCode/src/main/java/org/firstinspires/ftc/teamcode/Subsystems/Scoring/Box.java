package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helpers.Constants;

public class Box {

    private final HardwareMap hardwareMap;
    private Telemetry telemetry;
    public DistanceSensor pixelDetector;
    public Box(HardwareMap hardwareMap, Telemetry telemtry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
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

    public void loop() {
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
        telemetry.addLine("--- Pixel Detection ---");
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
    }



}