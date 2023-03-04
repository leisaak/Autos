package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.OpScript;

@TeleOp
public class ExampleTele extends OpScript {
    @Override
    public OpScript getInstance() {return this;}

    @Override
    public void initloop() {
        init_loop(bot);
    }

    @Override
    public void run() {
       // bot.rumble();
        //gamepad1.rumble(gamepad1.left_trigger, gamepad1.right_trigger, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        //SelfDriving.selfDriveDist(20);
        bot.PIDTUNER();
        telemetry.addLine("Example Tele Succeeded");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        runOpMode(getInstance());}
}
