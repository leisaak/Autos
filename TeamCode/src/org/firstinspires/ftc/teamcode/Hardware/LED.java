//package org.firstinspires.ftc.teamcode.Hardware;
//
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//
//import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
//
//public class LED extends HardwareHelper {
//    RevBlinkinLedDriver led;
//
//    public void lightsOn(RevBlinkinLedDriver.BlinkinPattern revBlinkinLedDriver){led.setPattern(revBlinkinLedDriver);}
//    public void lightsOff(){led.close();}
//    public LED(RevBlinkinLedDriver light) {
//        led = light;
//    }
//    public void chooseLights(){
//        if(bot.isOpModeRed()){lightsOn(RevBlinkinLedDriver.BlinkinPattern.RED);}
//        else if(bot.isOpModeBlue()){lightsOn(RevBlinkinLedDriver.BlinkinPattern.BLUE);}
//    }
//}
