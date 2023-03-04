package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.AbstractInit.Init.Motors;
import org.firstinspires.ftc.teamcode.Features.Config;

public class MotorData extends HardwareHelper {
    public void resetOdometers(){
        Motors.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors.odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors.odoMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motors.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors.odoMiddle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorPower(double power){
        Motors.leftFront.setVelocity(power * Config.encoderRatio);
        Motors.leftBack.setVelocity(power * Config.encoderRatio);
        Motors.rightBack.setVelocity(power * Config.encoderRatio);
        Motors.rightFront.setVelocity(power * Config.encoderRatio);
    }

    public void setMotorPowerRight(double power){
        Motors.leftFront.setVelocity(-power * Config.encoderRatio);
        Motors.leftBack.setVelocity(power * Config.encoderRatio);
        Motors.rightBack.setVelocity(-power * Config.encoderRatio);
        Motors.rightFront.setVelocity(power * Config.encoderRatio);
    }

    public void setMotorPowerLeft(double power){
        Motors.leftFront.setVelocity(power * Config.encoderRatio);
        Motors.leftBack.setVelocity(-power * Config.encoderRatio);
        Motors.rightBack.setVelocity(power * Config.encoderRatio);
        Motors.rightFront.setVelocity(-power * Config.encoderRatio);
    }

    public void setMotorPower(double left, double right){
        Motors.leftFront.setVelocity(left * Config.encoderRatio);
        Motors.leftBack.setVelocity(left * Config.encoderRatio);
        Motors.rightBack.setVelocity(right * Config.encoderRatio);
        Motors.rightFront.setVelocity(right * Config.encoderRatio);
    }

    public void setPower(double LFspeed, double LBspeed, double RFspeed, double RBspeed){
        Motors.leftFront.setVelocity(LFspeed * Config.encoderRatio);
        Motors.leftBack.setVelocity(LBspeed * Config.encoderRatio);
        Motors.rightBack.setVelocity(RBspeed * Config.encoderRatio);
        Motors.rightFront.setVelocity(RFspeed * Config.encoderRatio);
    }

    public void brake(){
        Motors.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.leftFront.setPower(0);
        Motors.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.leftBack.setPower(0);
        Motors.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.rightBack.setPower(0);
        Motors.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.rightFront.setPower(0);
    }

    public void setPowerBehavior(DcMotor.ZeroPowerBehavior Lbehavior, DcMotor.ZeroPowerBehavior Rbehavior ){
        Motors.leftFront.setZeroPowerBehavior(Lbehavior);
        Motors.leftBack.setZeroPowerBehavior(Lbehavior);
        Motors.rightBack.setZeroPowerBehavior(Rbehavior);
        Motors.rightFront.setZeroPowerBehavior(Rbehavior);
    }

    public void setRunMode(DcMotor.RunMode mode){
        Motors.leftFront.setMode(mode);
        Motors.leftBack.setMode(mode);
        Motors.rightBack.setMode(mode);
        Motors.rightFront.setMode(mode);
    }

    public double getVelocityRF(){return Motors.rightFront.getVelocity();}
    public double getVelocityRB(){return Motors.rightBack.getVelocity();}
    public double getVelocityLF(){return Motors.leftFront.getVelocity();}
    public double getVelocityLB(){return Motors.leftBack.getVelocity();}

    public double getLeftPosition(){return odoTicksToCm(Motors.odoLeft.getCurrentPosition());}
    public double getMiddlePosition(){return odoTicksToCm(Motors.odoMiddle.getCurrentPosition());}
    public static double odoTicksToCm(int ticks){return ticks * Config.encoderTicksTCM;}

    public MotorData() {
        Motors.rightFront.setDirection(DcMotor.Direction.FORWARD);
        Motors.rightBack.setDirection(DcMotor.Direction.FORWARD);
        Motors.leftFront.setDirection(DcMotor.Direction.REVERSE);
        Motors.leftBack.setDirection(DcMotor.Direction.REVERSE);
        Motors.odoLeft.setDirection(DcMotor.Direction.REVERSE);

        Motors.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motors.odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors.odoMiddle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Motors.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.odoMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motors.odoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        Motors.leftFront.setVelocityPIDFCoefficients(12,2.25,0.8,0.4);
//        Motors.leftBack.setVelocityPIDFCoefficients(12,2.25,0.8,0.4);
//        Motors.rightBack.setVelocityPIDFCoefficients(12,2.25,0.8,0.4);
//        Motors.rightFront.setVelocityPIDFCoefficients(12,2.25,0.8,0.4);
    }
}
