package org.firstinspires.ftc.teamcode.Helpers;

import org.firstinspires.ftc.teamcode.Features.Config;

public class bMath {
    public static double pi = 3.14159265359;
    public static double tau = 2 * pi;
    public static double sq2 = 1.41421356237309;


    //converts odometry ticks to cm moved
    public static double odoTicksToCm(int ticks){
        return ticks * Config.odoTicksToCm;
    }

    public static double sqd(double value) {
        return (value * value);
    }

    public static double squareInputWithSign(double input){
        double output = input * input;
        if (input < 0){
            output = output * -1;
        }
        return output;
    }
    //Clamp methods are to constrain values into a range
    public static double Clamp(double value, double min, double max) {
        double Ans = value;
        if (value >= max) {
            Ans = max;
        }
        if (value <= min) {
            Ans = min;
        }
        return Ans;


    }
    public static double Clamp(double value) { //Clamps between 1 and 0
        double Ans = value;
        if (value >= 1) {
            Ans = 1;
        }
        if (value <= 0) {
            Ans = 0;
        }
        return Ans;
    }
    public static long Clamp(long value, long min, long max) {
        long Ans = value;
        if (value >= max) {
            Ans = max;
        }
        if (value <= min) {
            Ans = min;
        }
        return Ans;
    }
    //End of clamp methods

    @Deprecated
    public static double mod(double value, double modulus) { //another way to do mod
        //mod(a,b) == a % b
        return value - Math.floor(value / modulus) * modulus;
    }

    /**
     * puts an input into the range between the min and the max.
     * modIntoRange(a, 0, b) == a % b
     * @param input the number you want put into the range
     * @param min the minimum value you want it to have
     * @param max the max value you want it to have
     * @return the number between min and max which is an integer multiple of the range
     * (max - min) away from the input
     */
    public static double modIntoRange (double input, double min, double max) {
        double ans = input;
        if(min < max){ //hopefully true
            double range = max - min;
            double helper1 = Math.ceil( (min - input) / range );
            ans = input + (helper1 * range);
        }
        if(max < min) { //this shouldn't happen, but if it does, don't worry about it
            ans = modIntoRange(input, max, min);
        }
        if(min == max) { //if they are exactly the same, just return that value
            ans = min;
        }
        return ans;
    }

    //cis(theta) = cos(theta) + ( i * sin(theta) )
    public static ComplexNum cis(double angle){
        ComplexNum Ans = new ComplexNum();
        Ans.real = Math.cos(angle);
        Ans.imag = Math.sin(angle);
        return Ans;
    }
    public static double acis(ComplexNum input){ //works, but inelegant
        double Ans = 0;
        if(input.imag == 0){//on real axis
            if(input.real > 0) return 0;
            if(input.real < 0) return pi;
            if(input.real == 0) return 0; //arbitrary choice, obviously there is no good Ans
        }
        if(input.real == 0){//on imaginary axis
            Ans = pi / 2 * sign(input.imag);
        }
        if(input.real > 0){ //the point is in quadrants 1 or 4
            Ans = Math.atan(input.imag / input.real);
        }
        if(input.real < 0){ //in quadrants 2 or 3
            Ans = Math.atan(input.imag / input.real) + Math.PI * sign(input.imag);
        }
        return Ans;
    }
    public static double acot(double input){ //verified
        return pi / 2 - Math.atan(input);
    }
    public static int sign(double input){
        if(input == 0){
            return 0;
        }
        return (input > 0 ? 1: -1);
    }
    public static double sign(double input, double max, double min){
        if(input == 0){
            return 0;
        }
        return (input > 0 ? max: min);
    }

    /**
     * puts an angle into the range -π to π
     * @param angleRadians the angle you want "regularized" in radians
     * @return the corresponding angle in the range -π to π
     */
    public static double regularizeAngleRad(double angleRadians){
        ComplexNum num = cis(angleRadians);
        double ans = acis(num);
        return ans;
    }

    /**
     * puts an angle into the range -180° to 180°
     * @param angleDegrees the angle you want "regularized" in degrees
     * @return the corresponding angle in the range -180° to 180°
     */
    public static double regularizeAngleDeg(double angleDegrees){
        double angleRad = Math.toRadians(angleDegrees);
        double ans = regularizeAngleRad(angleRad);
        ans = Math.toDegrees(ans);
        return ans;
    }

    /**
     * meant to allow help you find the shortest angle separating the angle you're
     * at from the angle you want
     * @param targetAngle the angle you want the robot to be at
     * @param currentAngle the current angle of the robot
     * @return an angle between -pi and pi that is the amount you should turn
     */
    public static double subtractAnglesRad(double targetAngle, double currentAngle){
        targetAngle = regularizeAngleRad(targetAngle);
        currentAngle = regularizeAngleRad(currentAngle);
        double ans = targetAngle - currentAngle;
        ans = regularizeAngleRad(ans);
        return ans;
    }


    /**
     * meant to allow help you find the shortest angle separating the angle you're
     * at from the angle you want
     * @param targetAngle the angle you want the robot to be at
     * @param currentAngle the current angle of the robot
     * @return an angle between -180 and 180 that is the amount you should turn
     */
    public static double subtractAnglesDeg(double targetAngle, double currentAngle){
        targetAngle = Math.toRadians(targetAngle);
        currentAngle = Math.toRadians(currentAngle);
        double ans = subtractAnglesRad(targetAngle, currentAngle);
        return Math.toDegrees(ans);
    }
    //toRadians and toDegrees are both in the standard Math class, so why did we make them again?
    @Deprecated
    public static double toRadians(double degrees) {
        return degrees * 0.0174;
    }
    @Deprecated
    public static double toDegrees(double radians) {
        return radians * 57.2957;
    }
}
