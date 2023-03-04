package org.firstinspires.ftc.teamcode.Features;

import org.firstinspires.ftc.teamcode.NewSelfDriving.Movement;
import org.firstinspires.ftc.teamcode.NewSelfDriving.PIDCoefficients;

/**
 * Class which stores all of our constants. All the variables are static meaning you can access them
 * directly from the class name/reference.
 * In here, find all the constants for elevator, TT, motors, odometers, etc.
 */
public class Config {
    //Odometers
    public static final int odoTicksPerRotation = 8192;
    public static final double odoDiameterIn = 2;
    public static final double odoDiameterCM = odoDiameterIn * 2.54;
    public static final double odoRadiusCM = 2.54;
    public static final double odoCircumferenceCM = odoDiameterCM * Math.PI;
    public static final double odoTicksToCm = odoCircumferenceCM / odoTicksPerRotation;

    //Motor names
    public static final String motorLF = "front_left_motor";
    public static final String motorLB = "back_left_motor";
    public static final String motorRF = "front_right_motor";
    public static final String motorRB = "back_right_motor";
    public static final String odoLeft = "enc_left";
    public static final String odoRight = "enc_right";
    public static final String odoMiddle = "enc_x";

    //Motor encoder
    public static final double encoderRatio = 2800d;
    public static final double WHEEL_DIAMETER = 4;
    public static final double INTER_WHEEL_WIDTH = 16;
    public static final double INTER_WHEEL_LENGTH = 14;
    public static final double TICKS_PER_DRIVE_ROTATION = 1120;
    public static final double ENCODER_WHEEL_DIAMETER = 2.0 * 2.54;
    public static final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    public static final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * ENCODER_WHEEL_DIAMETER;
    public static final double ENCODER_WIDTH = 12.0;
    public static final double MOTOR_TICKS_PER_REVOLUTION = 1120d; //my guess but we can try later
    public static final double encoderTicksTCM = ENCODER_WHEEL_CIRCUMFERENCE / MOTOR_TICKS_PER_REVOLUTION;

    //Deadzone
    public static final double DEADZONE = 0.15;

    //Square length
    public static final double squareDistance = 60;
    public static final double calibrate = 10;
    public static double squares(double square){return square*squareDistance;}

    //LED
    public static final String LED = "LED";

    //Motor Ticks
    public static int elevatorConeStack = -700;
    public static final double MOTOR_TICKS = 8192d;

    //Turn table
    public static final int turnTableLeft90 = 900;
    public static final int turnTableRight90 = -800;

    //Motor speed
    public static double speed = .55;


    //Self driving stuff
    public static PIDCoefficients drive = new PIDCoefficients(2,0,0, 0);
    public static PIDCoefficients driveX = new PIDCoefficients(1,0,0,0);
    public static PIDCoefficients driveY = new PIDCoefficients(2,0,0,0);
    public static PIDCoefficients driveTheta = new PIDCoefficients(3,0,0,0);
    public static PIDCoefficients turn = new PIDCoefficients(2.5,0,0,0);
    public static Movement driveAndMoveArm = new Movement(50,50,10, drive) {
        @Override
        public void runExtra() {}
    };
}
