package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Helpers.Counter;
import org.firstinspires.ftc.teamcode.Helpers.VelocityMath;

/**
 * Contains all the teleOp methods which eventually feed into Drivers
 * We have to driving methods: coordinate lock and regular mecanum
 * Coordinate lock uses the zero degree angle to keep as its "North". Thus, up the on joystick will always
 * the robot to zero degrees
 * Regular mecanum is based on the front of the robot. Forward will always be the way the robot is facing
 * In addition, we added a gear shift feature which increments the speed by a set amount. See Counter.
 */
public class Mecanum extends HardwareHelper {
    private double leftDiagPower = 0d, rightDiagPower = 0d, leftRotatePower = 0d, rightRotatePower = 0d;
    private final double sq2 = VelocityMath.sq2;
    //New mecanum
    public void coordinateLockMecanum(Gamepad gamepad, Counter scalePower){
        double imu_offset = Math.toRadians(-bot.angleDEG());
        double angle = (Math.toRadians(-bot.angleDEG()) - imu_offset);

        leftDiagPower = -(((gamepad.left_stick_y - gamepad.left_stick_x) / sq2 * Math.sin(angle) + ((gamepad.left_stick_y + gamepad.left_stick_x) / sq2) * Math.cos(angle)));
        rightDiagPower = -(((-(gamepad.left_stick_y + gamepad.left_stick_x) / sq2) * Math.sin(angle) + ((gamepad.left_stick_y - gamepad.left_stick_x) / sq2 * Math.cos(angle))));
        leftRotatePower = gamepad.right_stick_x;
        rightRotatePower = -gamepad.right_stick_x;

        bot.gearShift(scalePower);
        bot.setPowers(scalePower.getNum()*(leftDiagPower+leftRotatePower),
                scalePower.getNum()*(rightDiagPower+leftRotatePower),
                scalePower.getNum()*(rightDiagPower+rightRotatePower),
                scalePower.getNum()*(leftDiagPower+rightRotatePower) );
    }
    public void mecanumDrive(Gamepad gamepad, Counter scalePower){
        leftDiagPower = ((-gamepad.left_stick_y - gamepad.left_stick_x) / VelocityMath.sq2);
        rightDiagPower = ((-gamepad.left_stick_y + gamepad.left_stick_x) / VelocityMath.sq2);
        leftRotatePower = -gamepad.right_stick_x;
        rightRotatePower = gamepad.right_stick_x;

        bot.gearShift(scalePower);
        bot.setPowers(scalePower.getNum() *(leftDiagPower+leftRotatePower),
                scalePower.getNum() *(rightDiagPower+leftRotatePower),
                scalePower.getNum() *(rightDiagPower+rightRotatePower),
                scalePower.getNum() *(leftDiagPower+rightRotatePower));
    }
    //OLD MECANUM
    @Deprecated
    public void mecanumDriveExtra(Gamepad gamepad, Counter scalePower){
        double y = -gamepad.left_stick_y;
        double x = -gamepad.left_stick_x;
        double rx = -gamepad.right_stick_x;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double LF = (y + x + rx) / d;
        double LB = (y - x + rx) / d;
        double RF = (y - x - rx) / d;
        double RB = (y + x - rx) / d;

        bot.gearShift(scalePower);
        bot.setPowers(scalePower.getNum() * LF,
                scalePower.getNum() * LB,
                scalePower.getNum() * RF,
                scalePower.getNum() * RB);
    }
    public void coordinateLockExtra(Gamepad gamepad, Counter scalePower){
        double y = 0, x = 0, rx;
        boolean dpads = gamepad.dpad_down || gamepad.dpad_up;
        if(dpads){
            if(gamepad.dpad_up){y = scalePower.getNum();}
            else if(gamepad.dpad_down){y = -scalePower.getNum();}
        } else {
            y = -gamepad.left_stick_y;
            x = -gamepad.left_stick_x;
        }
        rx = -gamepad.right_stick_x;

        double rotX = x * Math.cos(bot.angleRAD()) - y * Math.sin(bot.angleRAD());
        double rotY = x * Math.sin(bot.angleRAD()) + y * Math.cos(bot.angleRAD());
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        bot.gearShift(scalePower);
        bot.setPowers(scalePower.getNum() * frontLeftPower,
                scalePower.getNum() * backLeftPower,
                scalePower.getNum() * frontRightPower,
                scalePower.getNum() * backRightPower);
    }
}
