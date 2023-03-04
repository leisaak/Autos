package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

@Autonomous
public class Red extends OpScript {
    @Override
    public OpScript getInstance() {return this;}

    @Override
    public void initloop() {init_loop(bot);}

    @Override
    public void run() {bot.coordinateDrive(Config.driveAndMoveArm);}

    @Override
    public void runOpMode() throws InterruptedException {runOpMode(getInstance());}
}
