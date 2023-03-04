package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.List;

public class BulkReader {
    List<LynxModule> allHubs;
    public HashMap<DcMotorEx, Integer> encoders;

    public BulkReader(HardwareMap hardwaremap){
        allHubs = hardwaremap.getAll(LynxModule.class);
        setAllHubsToManualBulkRead();
    }

    private void setAllHubsToManualBulkRead(){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clearCache(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}
