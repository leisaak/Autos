package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.OpScript;

@TeleOp
public class TeleopRed extends OpScript {

    @Override
    public OpScript getInstance() {return this;}

    @Override
    public void initloop() {
        init_loop(bot);
    }

    @Override
    public void run() {
        bot.Driver1();
        bot.Driver2();
        bot.Telemetry();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        runOpMode(getInstance());}
}