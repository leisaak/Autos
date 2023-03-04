package org.firstinspires.ftc.teamcode.Extra.disabled_samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.AbstractInit.Init;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.NewSelfDriving.Movement;
import org.firstinspires.ftc.teamcode.NewSelfDriving.PIDCoefficients;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

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

    public int[] prevTicks = new int[3];

    public double[] pose = new double[3];
//0 is y and 1 is x
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

    public void resetOdometry(double x, double y, double headingRadians){
        pose[0] = x;
        pose[1] = y;
        pose[2] = headingRadians;
        for (int i=0; i<3; i++) prevTicks[i] = encoders[i].getCurrentPosition();
    }

    public double angleDEG(){return -imuC.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public double angleRAD(){return -imuC.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
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
    public void drive(Movement movement){
        movement.getCoefficients().resetForPID();
        PIDCoefficients pidCoefficients = new PIDCoefficients(2,0,0,0);
        while(thetaCondition(movement) || xCondition(movement)){
            pose = updateOdometry();
            double dX = movement.getdX(), dY = movement.getdY(), dTheta = movement.getdTheta();

            xPID = movement.getCoefficients().getPID(dX - pose[1], dX, Config.speed);
            yPID = movement.getCoefficients().getPID(dY - pose[0], dY, Config.speed);
            rxPID = pidCoefficients.getPID(pidCoefficients, bMath.subtractAnglesDeg(dTheta, angleDEG()), dTheta, 0.3);

            movement.runExtra();
            drive(xPID, yPID, rxPID);
        }
    }
    public double subHead(Movement movement){return bMath.subtractAnglesDeg(movement.getdTheta(), angleDEG());}
    public double subX(Movement movement){return movement.getdX() - pose[1];}
    public void drive(double x, double y, double rx){
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
    boolean xCondition(Movement movement){return Math.abs(movement.getdX() - pose[1]) > 1;}
    boolean yCondition(Movement movement){return Math.abs(movement.getdY() - pose[0]) > 1;}
    boolean thetaCondition(Movement movement){return Math.abs(bMath.subtractAnglesDeg(movement.getdTheta(), angleDEG())) > 1;}

    public double getxPID(){return xPID;}

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
