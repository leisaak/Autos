package org.firstinspires.ftc.teamcode.SelfDriving;

import static org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper.bot;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

public class SelfTurning {
    double speed;
    double turnThreshold = Math.toRadians(0.75);//threshold
    double derivativeThreshold = 0.00005;
    double P = 0.11, I = 0.0, D = 0.0, F = 0.15;
    //See P and F description in SelfDriving
    double currentErrorRad;
    double derivative;
    double runningIntegral;
    double lastErrorRad, lastTime;
    double currentTime;

    public void selfTurn(double degree){
        turn(degree);
        bot.resetT();
        while(!checkForDone() && bot.opmodeIsActive()){turn(degree);}
        bot.brake();
        bot.setMotorPower(0);
    }

    void turn(double degree){
        Bot.telemetry.addLine("Turning to" + degree + "degrees now.");
        Bot.telemetry.addData("Current Position Degrees", bot.angleDEG());
        currentErrorRad = bMath.subtractAnglesRad(Math.toRadians(degree), bot.angleRAD());
        //subtracts desired angle by current
        currentTime = bot.getTime();
        derivative = findDerivative();
        updateIntegral();
        speed = updateSpeedPID();
        bot.setMotorPowerTurn(speed);
        updateLastValues();
        OpScript.update();
    }

    double updateSpeedPID(){
        double speed = 0;
        speed += P * currentErrorRad;//speed
        speed += I * runningIntegral;
        speed += D * derivative;
        if(Math.abs(currentErrorRad) > turnThreshold){
            //multiplies speed based on current error from target and F
            speed += bMath.sign(currentErrorRad) * F;
        }
        return Range.clip(speed, -.5, .5);
    }

    void updateLastValues(){
        lastTime = currentTime;
        lastErrorRad = currentErrorRad;
    }

    boolean checkForDone(){return (Math.abs(currentErrorRad) < turnThreshold) && (Math.abs(derivative) < derivativeThreshold);}
    //returns true or false based on if we are under our threshold
    double findDerivative(){return (currentErrorRad - lastErrorRad) / (currentTime - lastTime);}
    void updateIntegral(){runningIntegral += ((currentErrorRad + lastErrorRad) / 2.0) * (currentTime - lastTime);}
}
