package org.firstinspires.ftc.teamcode.SelfDriving;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Helpers.bMath;

public class PID extends HardwareHelper {
    private double P = 1, I = 0.16, D = 0.2;
    public static double[] movePID = {1, 0.3, 0.2};
    public static double[] turnPID = {2.5,.3,.1};
    private double errorSum, lastError, dError; // Accumulator for integral term and previous error
    public static double lastTime;
    boolean first = true;
    public static double timeSum = 0;
    public static int timeDifference = 0;

    public void setPID(double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
    }

    /**
     * @param pid: double array to set PID
     */
    public void setPID(double[] pid){
        this.P = pid[0];
        this.I = pid[1];
        this.D = pid[2];
    }
    public void resetPID(double[] pid){
        this.P = pid[0];
        this.I = pid[1];
        this.D = pid[2];
    }

    /**
     * Resets the PID before running
     */
    public void resetForPID(){
        errorSum = 0;
        dError = 0;
        first = true;
        bot.resetOdometers();
    }

    /**
     * Updates the speed the robot should go at. Use inside of a loop until target is reached
     * @param error: error from target
     * @return: the speed at which the robot should go based off of PID calculations
     */
    public double speed(double error, double speed){
        double currentTime = System.nanoTime();
        errorSum += (error / SelfDriving.targetDistance) * ((currentTime - lastTime) / 1000000000); // Update integral accumulator
        dError = (((currentTime - lastTime) / 1000000000) > .001) ? ((error / SelfDriving.targetDistance) - lastError) / ((currentTime - lastTime) / 1000000000) : 0;
        Bot.telemetry.addData("error", error);
        Bot.telemetry.addData("time", currentTime);
        Bot.telemetry.addData("last error", lastError);
        Bot.telemetry.addData("last time", lastTime);
        Bot.telemetry.addData("P ",  P * (error / SelfDriving.targetDistance));
        Bot.telemetry.addData("I", I * errorSum);
        Bot.telemetry.addData("D", D * dError);

        if(first){
            dError = 0;
            SelfDriving.first = (P * (error / SelfDriving.targetDistance) + I * errorSum + D * dError) * speed;
            SelfDriving.FistCT = D * dError;
            SelfDriving.firstT = (currentTime - lastTime) / 1000000000;
            first = false;
        }

        timeSum += (currentTime - lastTime);
        timeDifference++;

        lastError = error / SelfDriving.targetDistance; // Update previous error
        lastTime = currentTime;

        // Calculate control output
        return (P * (error / SelfDriving.targetDistance) + I * errorSum + D * dError) * speed;
    }

    public double speed(double error, double min, double max, double speed){
        errorSum += error; // Update integral accumulator
        dError = error - lastError; // Calculate derivative
        lastError = error; // Update previous error

        // Calculate control output
        return (P * error + I * errorSum + D * dError) * speed;

    }

    public double derivative(double error){return error - lastError;}
    public double errorDrive(){return SelfDriving.targetDistance - bot.getLeftOdoPos();}
    public double errorStrafe(){return Math.abs(SelfDriving.targetDistance) - Math.abs(bot.getMiddleOdoPos());}
    public double errorTurn(){return bMath.subtractAnglesDeg(SelfDriving.targetDistance, bot.angleDEG());}
    public double errorTurnDeg(){
        if(Math.abs((720-bot.angleDEG()+SelfDriving.targetDegrees)%360)<(Math.abs((720-SelfDriving.targetDegrees+ bot.angleDEG())%360))) {
            return -Math.abs((720-bot.angleDEG()+SelfDriving.targetDegrees)%360);
        } else {
            return Math.abs((720-SelfDriving.targetDegrees+ bot.angleDEG())%360);
        }
    }
}
