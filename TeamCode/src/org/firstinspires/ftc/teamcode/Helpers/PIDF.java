package org.firstinspires.ftc.teamcode.Helpers;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;

public class PIDF extends HardwareHelper {
    private static double currentError;
    private static double runningIntegral;
    private static double derivative;
    private static double threshold = 3;
    private static double currentTime, lastTime, lastError;
    static double Pgain = 0.019, Igain = 0.0, Dgain = 0.0, Fgain = 0.0230;
    private static final double derivativeThreshold = 0.0005;
    private static double P = 0, I = 0, D = 0, F = 0;
    private static double error =0d;
    private static double time =0d;


    public PIDF(double P, double  I, double D, double F){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
    }
    public PIDF(){
        P = 0.019;
        I = 0d;
        D = 0d;
        F = 0.230;
    }

    public static double pid(double error){
        time = bot.getTime();
        currentError = error;
        derivative = findDerivative();

        double speed = 0;
        //TODO mathematically find the gain values which offset the PID
        speed += currentError;
        speed += runningIntegral;
        speed += derivative;
        return speed;
    }

    public static double pid(double error, double min, double max){
        currentTime = bot.getTime();
        currentError = error;
        derivative = findDerivative();
        updateIntegral();

        double speed = 0;
        //TODO mathematically find the gain values which offset the PID
        speed += currentError;
        speed += runningIntegral;
        speed += derivative;
        if(Math.abs(currentError) > threshold){
            speed += bMath.sign(currentError, max, min) * F;
        }
        updateLastValues();
        return speed;
    }

    static void updateLastValues(){
        //Gets values from last time which find deriv and error
        lastTime = currentTime;
        lastError = currentError;
    }
    static double findDerivative(){return (currentError - lastError) / (currentTime - lastTime);}
    static void updateIntegral(){runningIntegral += ((currentError + lastError) / 2.0) * (currentTime - lastTime);}

    public static boolean conditions(){return (Math.abs(currentError) < threshold) && (Math.abs(derivative) < derivativeThreshold);}
}
