package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Cache {

    private final HardwareMap hardwareMap;
    public List<LynxModule> allHubs;
    public LynxModule CONTROL_HUB;
    public LynxModule EXPANSION_HUB;

    public Cache(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        // Enable Bulk Caching
        //This mode enables the user's code to determine the best time to refresh the cached bulk-read data.
        //Well organized code can place all the sensor reads in one location, and then just reset the cache once per control cycle
        //This should get us faster responses as we have control over when the cache resets.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);

        //Debugging purposes ^ -- We can telemetry log the AMPs from each sensor/motor to see if a specific motor is draining too much power
        //Which can be a hardware related reason or software reason.
    }

    public void loop(Telemetry telemetry) {
        allHubs.forEach(LynxModule::clearBulkCache); //Reset Bulk Cache
        telemetry.addLine("--- Bulk Cache ---");
        telemetry.addLine("Bulk Cache Reset");
    }

}