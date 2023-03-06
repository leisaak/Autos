package org.firstinspires.ftc.teamcode.Extra.disabled_samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.AbstractInit.Init;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.NewSelfDriving.Function;
import org.firstinspires.ftc.teamcode.NewSelfDriving.Movement;
import org.firstinspires.ftc.teamcode.NewSelfDriving.PIDCoefficients;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleUnaryOperator;

import virtual_robot.util.AngleUtils;

/**
 * Utility class that represents a robot with mecanum drive wheels and three "dead-wheel" encoders.
 */
public class EncBot extends HardwareHelper {
    double xPID, yPID, rxPID;
    public final double WHEEL_DIAMETER = 4;
    public final double INTER_WHEEL_WIDTH = 16;
    public final double INTER_WHEEL_LENGTH = 14;
    public final double TICKS_PER_DRIVE_ROTATION = 1120;
    public final double TICKS_PER_ENCODER_ROTATION = 1120;
    public final double ENCODER_WHEEL_DIAMETER = 2;
    private final double ENCODER_TICKS_PER_REVOLUTION = 1120;
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0;
    private final double ENCODER_WIDTH = 12.0;

    public final DcMotorEx[] motors = new DcMotorEx[4]; //back_left, front_left, front_right, back_right
    public final DcMotorEx[] encoders = new DcMotorEx[3]; //right, left, X
    IMU imuC;

    //stores previous ticks inside updateOdometry() to add to rolling sum of location
    public int[] prevTicks = new int[3];
    /**
     * Stores the odometry for all three encoders. Currently we only need objects 0-1 since 2 was to calculte heading.
     * pose[1] stores x position and pose[0] stores y position
     */
    public double[] pose = new double[3];

    public void init(HardwareMap hwMap){
        imuC = hwMap.get(IMU.class, "imu");
        //instantiating the previously null objects
        imuC.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imuC.resetYaw();
        String[] motorNames =  new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i=0; i<4; i++) motors[i] = hwMap.get(DcMotorEx.class, motorNames[i]);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        String[] encoderNames = new String[]{"enc_right", "enc_left", "enc_x"};
        for (int i=0; i<3; i++) encoders[i] = hwMap.get(DcMotorEx.class, encoderNames[i]);
    }
    public void in(){
        DcMotorEx[] storeEx = new DcMotorEx[3];
        storeEx[0] = Init.Motors.odoRight;
        storeEx[1] = Init.Motors.odoLeft;
        storeEx[2] = Init.Motors.odoMiddle;
        for (int i=0; i<3; i++) encoders[i] = storeEx[i];
    }

    public void setDrivePower(double px, double py, double pa){
        double[] p = new double[4];
        p[0] = -px + py - pa;
        p[1] = px + py - pa;
        p[2] = -px + py + pa;
        p[3] = px + py + pa;
        double max = Math.max(1, Math.max(Math.abs(p[0]), Math.max(Math.abs(p[1]), Math.max(Math.abs(p[2]), Math.abs(p[3])))));
        if (max > 1) for (int i=0; i<4; i++) p[i] /= max;
        for (int i=0; i<4; i++) motors[i].setPower(p[i]);
    }

    /**
     *
     * @param x x cooridinate to restart robot
     * @param y y coordinate to restart robot
     * @param headingDeg heading in degrees to restart robot
     */
    public void resetOdometry(double x, double y, double headingDeg){
        pose[0] = x;
        pose[1] = y;
        pose[2] = Math.toRadians(headingDeg);
        for (int i=0; i<3; i++) prevTicks[i] = encoders[i].getCurrentPosition();
    }

    public double angleDEG(){return -imuC.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public double angleRAD(){return -imuC.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    /**
     * This method updates the odometry. It keeps a running calculation of the x and y position
     * by checking the change of distance from last iteration
     * @return returns a double array. 0=y, 1=x, 2="no need"
     */
    public double[] updateOdometry(){
        int[] ticks = new int[3];
        for (int i=0; i<3; i++) ticks[i] = encoders[i].getCurrentPosition();
        int newRightTicks = ticks[0] - prevTicks[0];
        int newLeftTicks = ticks[1] - prevTicks[1];
        int newXTicks = ticks[2] - prevTicks[2];
        prevTicks = ticks;
        double rightDist = newRightTicks * (ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION);
        double leftDist = -newLeftTicks * (ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION);
        double dyR = 0.5 * (rightDist + leftDist);
        double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;
        double dxR = -newXTicks * (ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION);
        double avgHeadingRadians = pose[2] + headingChangeRadians / 2.0;
        double cos = Math.cos(angleRAD());
        double sin = Math.sin(angleRAD());
        pose[0] += -dxR*sin + dyR*cos;
        pose[1] += dxR*cos + dyR*sin;
        pose[2] = AngleUtils.normalizeRadians(pose[2] + headingChangeRadians);
        return pose;
    }
    /**
     * This is driving method that goes to x,y, and theta with tuned PID.
     * Input a movement object that has that data and the method goes there.
     * To drive there it uses mecanum driving code tht takes in x, y, and turning input. To drive there,
     * there is PID for each position which then scales speed and where to go.
     * Lastly, movement has an abstract method called runExtra() which is ran in the method--use to run other stuff.
     * @param movement abstract object that has x,y, theta, and PID values that are used
     */
    public void drive(Movement movement){
        movement.getCoefficients().resetForPID();//resetting value to begin loop
        while(thetaCondition(movement) || xCondition(movement) ||  yCondition(movement)){
            pose = updateOdometry();//updating odometers
            double dX = movement.getdX(), dY = movement.getdY(), dTheta = movement.getdTheta();

            //updating PID values for x, y and theta
            xPID = movement.getCoefficients().getPID(dX - pose[1], Math.abs(dX), Config.speed);
            yPID = movement.getCoefficients().getPID(dY - pose[0], Math.abs(dY), Config.speed);
            rxPID = Config.turn.getPID(Config.turn, bMath.subtractAnglesDeg(dTheta, angleDEG()), dTheta, 0.4);

            movement.runExtra();//run extra if needed
            drive(xPID, yPID, rxPID);//drive there
        }
    }

    /**
     * AutoDrive method that takes in a movement object and function object. Supposed to follow path
     * which the function input designates
     * @param movement abstract object that has x,y, theta, and PID values that are used
     * @param function lamba expression to input how the robot should drive
     */
    public void drive(Movement movement, Function function){
        Function fx = function.derivative();//function in terms of x
        Function fy = function.derivative().changeVariable();//function in terms of y
        //creating new x and y PID objects
        PIDCoefficients xpid = new PIDCoefficients(1,0,0,0);
        PIDCoefficients ypid = new PIDCoefficients(1,0,0,0);
        movement.getCoefficients().resetForPID();
        while(thetaCondition(movement) || xCondition(movement) ||  yCondition(movement)){
            pose = updateOdometry();
            double dX = movement.getdX(), dY = movement.getdY(), dTheta = movement.getdTheta();
            xpid.setP( movement.getCoefficients().getP() * fx.evaluate(pose[1]));//scaling P value based on x
            ypid.setP( movement.getCoefficients().getP() * fy.evaluate(pose[0]));//scaling P value based on y

            xPID = xpid.getPID(dX - pose[1], Math.abs(dX), Config.speed);
            yPID = ypid.getPID(dX - pose[1], Math.abs(dY), Config.speed);
            rxPID = Config.turn.getPID(Config.turn, bMath.subtractAnglesDeg(dTheta, angleDEG()), dTheta, 0.3);

            movement.runExtra();
            drive(xPID, yPID, rxPID);
        }
    }
    /**
     * Driving code to go to position. It is essentially mecanum driving but uses PID to drive there.
     * All inputs are scaled to be between -1 and 1 since power goes to motors
     * @param x x input
     * @param y y input
     * @param rx turn input
     */
    private void drive(double x, double y, double rx){
        double rotX = x * Math.cos(angleRAD()) - y * Math.sin(angleRAD());
        double rotY = x * Math.sin(angleRAD()) + y * Math.cos(angleRAD());
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motors[0].setPower(backLeftPower);
        motors[1].setPower(frontLeftPower);
        motors[2].setPower(frontRightPower);
        motors[3].setPower(backRightPower);
    }

    /**
     * X position condition for loop
     * @param movement object to access desired X
     * @return true or false if robot has reached its dX
     */
    boolean xCondition(Movement movement){return Math.abs(movement.getdX() - pose[1]) > 0.25;}
    /**
     * Y position condition for loop
     * @param movement object to access desired Y
     * @return true or false if robot has reached its dY
     */
    boolean yCondition(Movement movement){return Math.abs(movement.getdY() - pose[0]) > 0.25;}
    /**
     * Theta position condition for loop. Uses subtractingAnglesDeg to calculate error
     * @param movement object to access desired theta
     * @return true or false if robot has reached its dTheta
     */
    boolean thetaCondition(Movement movement){return Math.abs(bMath.subtractAnglesDeg(movement.getdTheta(), angleDEG())) > 1;}


    public double subHead(Movement movement){return bMath.subtractAnglesDeg(movement.getdTheta(), angleDEG());}
    public double subX(Movement movement){return movement.getdX() - pose[1];}
    public double subY(Movement movement){return movement.getdY() - pose[0];}

    public double getxPID(){
        return xPID;
    }

    public double getyPID() {
        return yPID;
    }

    public double getRxPID() {
        return rxPID;
    }

    public double[] getPose(){
        return pose;
    }
    public void setPose(double[] newPose){pose = newPose;}
}
