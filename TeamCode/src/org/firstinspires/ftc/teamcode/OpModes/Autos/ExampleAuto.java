package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.OpScript;
import org.firstinspires.ftc.teamcode.SelfDriving.SelfDriving;

@Autonomous
public class ExampleAuto extends OpScript {
    @Override
    public OpScript getInstance() {return this;}

    @Override
    public void initloop() {
        init_loop(bot);
    }

    @Override
    public void run() {
        bot.turn(-90);
        telemetry.addData("test", bot.getLeftOdoPos());
        telemetry.addData("test 2", bot.getMiddleOdoPos());
        telemetry.addLine("Example Auto Succeeded!");
        telemetry.addData("test", SelfDriving.checkTurn());
    }

    @Override
    public void runOpMode() throws InterruptedException {runOpMode(getInstance());}
}
