package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Annotations.Important;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Helpers.BulkReader;
import org.firstinspires.ftc.teamcode.SelfDriving.PID;
import org.firstinspires.ftc.teamcode.SelfDriving.SelfDriving;

/**
 * Abstract class which should be the inheritance class for every OpMode
 * Run(): input whatever you want to loop during the opMode
 * initloop(): loops the init (basically for camera and pre-run)
 */
@Important
public abstract class OpScript extends LinearOpMode {
    public Bot bot;
    public boolean runAuto = true;
    public int barcode = -1;
    public BulkReader bulkReader;

    public abstract void run();//method where you put wherever needs to be looped
    public abstract void initloop();//loops whatever needs to happen in init
    public abstract OpScript getInstance();//init

    /**
     * runs the opMode
     * @param opmode: input the OpSript object to run
     */
    public void runOpMode(OpScript opmode) {
        opmode.bot = Bot.getInstance(opmode);
        HardwareHelper.bot = Bot.getInstance();
        while (!opmode.opModeIsActive() && !opmode.isStarted()) {opmode.initloop();}
        while (opmode.opModeIsActive() && opmode.isStarted() && bot.isRunning()) {
            opmode.run();
            opmode.telemetry.update();
            opmode.bulkReader.clearCache();
            Bot.cycleNumber++;
            Bot.runAuto = false;
        }
    }

    /**
     * Used to loop the camera in auto and telemetry pre-run
     * @param bot: input the Bot objecct to run
     */
    public static void init_loop(Bot bot) {
        //Auto: scan QR code. TeleOp: telemetry
        if(bot.isAuto()){
            Bot.telemetry.addData("QR Code num", Bot.barcode);
        }
        Bot.telemetry.addData("IMU:", bot.angleDEG());
        Bot.telemetry.addData("Voltage", bot.getVoltage());
        Bot.telemetry.update();
    }

    public static void update() {
        Bot.bulkReader.clearCache();
        Bot.telemetry.update();
    }
}
