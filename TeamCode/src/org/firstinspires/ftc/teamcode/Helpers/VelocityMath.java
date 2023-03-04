package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Features.Config;

public class VelocityMath {

    //TRASH CLASSS FOR NOW
    public static double leftDiagPower = 0;
    public static double rightDiagPower = 0;
    public static double leftRotatePower = 0;
    public static double rightRotatePower = 0;
    public static double scalePower = 1;
    public static double angle = 0;
    public static boolean coordinate_system_lock = true;
    public static double imu_offset = 0;
    public static boolean coordinate_system_lock_transition_controller = true;
    public static final double sq2 = Math.sqrt(2);
    public static final double encoderRatio = 2800d;

    public static double FrontLeftVelocity(Gamepad gamepad){
        leftDiagPower = Config.encoderRatio*((-gamepad.left_stick_y + gamepad.left_stick_x) / bMath.sq2);
        leftRotatePower = Config.encoderRatio*gamepad.right_stick_x;
        scalePower = gamepad.left_trigger;
        return (scalePower*(leftDiagPower+leftRotatePower));
    }
    public static double FrontRightVelocity(Gamepad gamepad){
        rightDiagPower = Config.encoderRatio*((-gamepad.left_stick_y - gamepad.left_stick_x) / bMath.sq2);
        rightRotatePower = Config.encoderRatio*-gamepad.right_stick_x;
        scalePower = gamepad.left_trigger;
        return scalePower*(rightDiagPower+rightRotatePower);
    }
}
