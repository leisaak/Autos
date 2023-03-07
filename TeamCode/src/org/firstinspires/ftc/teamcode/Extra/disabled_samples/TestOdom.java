package org.firstinspires.ftc.teamcode.Extra.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.NewSelfDriving.CreatePoints;
import org.firstinspires.ftc.teamcode.NewSelfDriving.Function;
import org.firstinspires.ftc.teamcode.NewSelfDriving.Movement;
import org.firstinspires.ftc.teamcode.NewSelfDriving.PathBuilder;

/**
 * TeleOp op mode to test odometry with three "dead-wheel" encoders. This op mode will work with
 * either the MecBot or the XDriveBot robot configuration.
 */
@TeleOp(name = "TestOdom", group = "OdomBot")
public class TestOdom extends LinearOpMode {

    EncBot bot = new EncBot();
    Function Squared = new Function(x -> (x * x));
    Function oneHalf = new Function(x -> x);
    Function xCubed = new Function(x -> Math.pow((1 / 3) * x, 3));
    Function yCubedInv = new Function(x -> 3 * Math.pow(x, -3));
    PathBuilder.Extra arm = () -> {
        telemetry.addData("xpid:", bot.getxPID());
        telemetry.addData("ypid:", bot.getyPID());
        telemetry.addData("rxpid:", bot.getRxPID());
        telemetry.addData("POSE", "y = %.1f  x = %.1f  h = %.1f", bot.getPose()[0], bot.getPose()[1],
                bot.angleDEG());
        telemetry.update();
    };
    double[] pose;
    Movement drive = new Movement(10,30,0, Config.drive) {
        @Override
        public void runExtra() {telemetry(drive);}
    };
    Movement drive2 = new Movement(10,27,90, Config.drive) {
        @Override
        public void runExtra() {telemetry(drive2);}
    };
    Movement drive3 = new Movement(50,50,35, Config.drive) {
        @Override
        public void runExtra() {telemetry(drive3);}
    };
    Movement drive4 = new Movement(-30,-30,45, Config.drive) {
        @Override
        public void runExtra() {telemetry(drive4);}
    };
    public void telemetry(Movement movement){
        telemetry.addData("xpid:", bot.getxPID());
        telemetry.addData("ypid:", bot.getyPID());
        telemetry.addData("rxpid:", bot.getRxPID());
        telemetry.addData("sub heading:", bot.subHead(movement));
        telemetry.addData("sub x:", bot.subX(movement));
        telemetry.addData("POSE", "y = %.1f  x = %.1f  h = %.1f", bot.getPose()[0], bot.getPose()[1],
                bot.angleDEG());
        telemetry.update();
    }
    public void runOpMode(){
        CreatePoints.addY(oneHalf, 2,0,25);
        CreatePoints.addX(oneHalf, 2,0,7);
        System.out.println(CreatePoints.getX());
        System.out.println(CreatePoints.getY());
        PathBuilder path1 = new PathBuilder(PathBuilder.createPath(CreatePoints.getX(),CreatePoints.getY(),90, Config.drive, arm));
        bot.init(hardwareMap);
        bot.resetOdometry(0, 0, 0);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("time", this.time);
            telemetry.update();
        }

        int count = 0;
        while (opModeIsActive()){
            if(count<1){
                bot.drive(path1);
//                bot.drive(drive);
//                bot.drive(drive2);
//                bot.drive(drive3);
//                bot.drive(drive4);
            }
            count++;
            pose = bot.updateOdometry();
            telemetry.addData("time", this.time);
            telemetry.addData("POSE", "y = %.1f  x = %.1f  h = %.1f", pose[0], pose[1],
                    Math.toDegrees(pose[2]));
            telemetry.addData("Controller", "x = %.1f  y = %.1f  h = %.1f", gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x);
            telemetry.addData("Back Left", "T = %d  V = %.0f", bot.motors[0].getCurrentPosition(), bot.motors[0].getVelocity());
            telemetry.addData("Front Left", "T = %d  V = %.0f", bot.motors[1].getCurrentPosition(), bot.motors[1].getVelocity());
            telemetry.addData("Front Right", "T = %d  V = %.0f", bot.motors[2].getCurrentPosition(), bot.motors[2].getVelocity());
            telemetry.addData("Back Right", "T = %d  V = %.0f", bot.motors[3].getCurrentPosition(), bot.motors[3].getVelocity());
            telemetry.addData("Right Enc", "T = %d  V = %.0f", bot.encoders[0].getCurrentPosition(), bot.encoders[0].getVelocity());
            telemetry.addData("Left Enc", "T = %d  V = %.0f", bot.encoders[1].getCurrentPosition(), bot.encoders[1].getVelocity());
            telemetry.addData("X Enc", "T = %d  V = %.0f", bot.encoders[2].getCurrentPosition(), bot.encoders[2].getVelocity());
            telemetry.update();
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.right_stick_x;
            bot.setDrivePower(px, py, pa);
        }
    }
}
