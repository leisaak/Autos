package org.firstinspires.ftc.teamcode.Helpers;

import java.util.Random;

/**
 * like the {@code Math} class, but for complex numbers
 * there's a lot of stuff in here that is not useful to the robotics team, but I like the
 * completeness of having it all
 * @author Joseph
 *
 */
public final class Cmath {
    /**
     * Don't let anyone instantiate this class.
     */
    private Cmath() {}

    public static final double pi = 3.14159265358979323846;


    //basic arithmetic

    /**
     * adds 2 complex numbers
     * @param a the first complex number
     * @param b the second complex number
     * @return a + b
     */
    public static ComplexNum add(ComplexNum a, ComplexNum b){
        ComplexNum Ans = new ComplexNum();
        Ans.real = a.real + b.real;
        Ans.imag = a.imag + b.imag;
        return Ans;
    }

    /**
     * subtracts one complex number from another complex number
     * @param a the first complex number
     * @param b the second complex number
     * @return a - b
     */
    public static ComplexNum subtract(ComplexNum a, ComplexNum b){ //do a - b
        ComplexNum Ans = new ComplexNum();
        Ans.real = a.real - b.real;
        Ans.imag = a.imag - b.imag;
        return Ans;
    }

    /**
     * multiplies 2 complex numbers
     * @param a the first complex number
     * @param b the second complex number
     * @return a * b
     */
    public static ComplexNum multiply(ComplexNum a, ComplexNum b){
        ComplexNum Ans = new ComplexNum();
        Ans.real = (a.real * b.real) - (a.imag * b.imag);
        Ans.imag = (a.imag * b.real) + (a.real * b.imag);
        return Ans;
    }

    /**
     * multiplies a complex number by a {@code double}
     * @param a the complex number
     * @param b the double
     * @return a * b
     */
    public static ComplexNum multiply(ComplexNum a, double b){
        ComplexNum ans = a.copy();
        ans.real *= b;
        ans.imag *= b;
        return ans;
    }

    /**
     * divides one complex number by another complex number. Note:
     * <li> dividing by 0 + 0i will give a runtime error
     * <p>
     * @param a the first complex number
     * @param b the second complex number
     * @return a / b
     */
    public static ComplexNum divide(ComplexNum a, ComplexNum b) { //verified
        ComplexNum ans = new ComplexNum();
        double denominator = (b.real * b.real) + (b.imag * b.imag);
        ans.real = ( (a.real * b.real) + (a.imag * b.imag) ) / denominator;
        ans.imag = ( (a.imag * b.real) - (a.real * b.imag) ) / denominator;
        return ans;
    }

    /**
     * divides a complex number by a {@code double}
     * @param a the complex number
     * @param b the {@code double}
     * @return a / b
     */
    public static ComplexNum divide(ComplexNum a, double b){
        ComplexNum ans = a.copy();
        ans.real /= b;
        ans.imag /= b;
        return ans;
    }

    /**
     * divides a {@code double} by a complex number
     * @param a the {@code double}
     * @param b the complex number
     * @return a / b
     */
    public static ComplexNum divide(double a, ComplexNum b){
        ComplexNum ans = new ComplexNum();
        double denominator = (b.real * b.real) + (b.imag * b.imag);
        ans.real =  a * b.real / denominator;
        ans.imag = -a * b.imag / denominator;
        return ans;
    }


    //exponents and logarithms

    /**
     * raises a complex number to the power of another complex number
     * @param base
     * @param exponent
     * @return base <sup> exponent </sup>
     */
    public static ComplexNum power(ComplexNum base, ComplexNum exponent){
        return exp(multiply(ln(base), exponent));
    }

    /**
     * raises one complex number to the power of a {@code double}
     * @param base
     * @param exponent
     * @return base <sup> exponent </sup>
     */
    public static ComplexNum power(ComplexNum base, double exponent){
        return exp(multiply(ln(base), exponent));
    }

    /**
     * finds the log base one complex number of another complex number
     * @param base the base you want the log function to be in
     * @param Z the number you want the log of
     * @return log <sub >base</sub>(Z)
     */
    public static ComplexNum log(ComplexNum base, ComplexNum Z) {
        return divide( ln(Z), ln(base) );
    }

    /**
     * returns Euler's number e raised to the power of a complex number
     * @param input the exponent to raise e to
     * @return the value of e^input, where e is the base of the natural logarithm
     */
    public static ComplexNum exp(ComplexNum input){ //verified
        ComplexNum Ans = cis(input.imag);
        Ans.timesEquals(Math.exp(input.real));
        return Ans;
    }

    /**
     * finds the natural log of a complex number
     * @param input the complex number you wish to take the natural log of
     * @return the simplest natural log of the input
     * @note There are an infinite number of possible natural logs for any given
     * complex number, this method returns the one with an imaginary part on
     * the interval (-π, π]
     */
    public static ComplexNum ln(ComplexNum input){ //verified
        double imag = angle(input);
        double vectorLengthSquared = bMath.sqd(input.imag) + bMath.sqd(input.real);
        double real = 0.5 * Math.log(vectorLengthSquared);
        return new ComplexNum(real, imag);
    }

    //square and square root

    /**
     * multiplies a given complex number by itself
     * @param input the number you want to square
     * @return input {@code *} input
     */
    public static ComplexNum square(ComplexNum input) {
        return new ComplexNum(
                input.real * input.real - input.imag * input.imag,
                2 * input.real * input.imag
        );
    }

    /**
     * finds the square root of a complex number
     * @param input the number you want the square root of
     * @return input <sup> 1/2 </sup>
     */
    public static ComplexNum sqrt(ComplexNum input) {
        return power(input, 0.5);
    }

    //trigonometric functions

    /**
     * creates a new complex number on the unit circle at a specified angle
     * @param angle the angle you want the new complex number to have
     * @return the new complex number
     */
    public static ComplexNum cis(double angle){
        ComplexNum Ans = new ComplexNum();
        Ans.real = Math.cos(angle);
        Ans.imag = Math.sin(angle);
        return Ans;
    }

    /**
     * finds the sine of a complex number
     * @param input the number you want the sine of
     * @return sine of input
     */
    public static ComplexNum sin(ComplexNum input) { //verified
        ComplexNum ans = input.timesI();
        ans.equals(exp(ans))
                .minusEquals(divide(1.0, ans))
                .divideEquals(2.0)
                .timesEquals(-1.0)
                .timesEqualsI();
        return ans;
    }

    /**
     * finds the cosine of a complex number
     * @param input the number you want the cosine of
     * @return cosine of input
     */
    public static ComplexNum cos(ComplexNum input) { //verified
        ComplexNum ans = input.timesI();
        ans.equals(exp(ans))
                .plusEquals(divide(1.0, ans))
                .divideEquals(2.0);
        return ans;
    }

    /**
     * finds the tangent of a complex number
     * @param input the number you want to find the tangent of
     * @return the tangent of the input
     */
    public static ComplexNum tan(ComplexNum input) { //verified
        ComplexNum helper1 = exp(input.timesI());
        ComplexNum helper2 = divide(1.0, helper1);
        ComplexNum numerator = subtract(helper1, helper2);
        ComplexNum denominator = add(helper1, helper2).timesI();
        return divide(numerator, denominator);
    }

    //random
    private static final class RandomNumberGenerator {
        static final Random random = new Random();
        static double nextDouble(double min, double max) {
            double range = max - min;
            return random.nextDouble() * range + min;
        }
        static ComplexNum nextComplexNum() {
            return new ComplexNum(random.nextDouble(), random.nextDouble());
        }
        static ComplexNum nextComplexNum(double min, double max) {
            double real = RandomNumberGenerator.nextDouble(min, max);
            double imag = RandomNumberGenerator.nextDouble(min, max);
            return new ComplexNum(real, imag);
        }
    }

    /**
     * creates a new complex number with real and imaginary components in the range 0 - 1
     * @return the new random ComplexNum
     */
    public static ComplexNum random() {
        return RandomNumberGenerator.nextComplexNum();
    }

    /**
     * creates a new complex number with real and imaginary components in the range min - max
     * @return the new random ComplexNum
     */
    public static ComplexNum random(double min, double max) {
        return RandomNumberGenerator.nextComplexNum(min, max);
    }

    //miscellaneous math specific to complex numbers

    /**
     * finds the angle of a complex number off the real axis. Note:
     * <li> the result is always between <i> -pi </i> and <i> pi </i>
     * <li> if the real component is negative and the imaginary component is 0,
     * this returns <i> pi </i>
     * <li> if the complex number is 0 + 0i, this returns 0
     * <p>
     * @param input
     * @return the angle of the line which connects the input to the origin
     */
    public static double angle(ComplexNum input){ //works, but inelegant
        double ans = 0;
        if(input.imag == 0){ //on real axis
            if(input.real > 0) return 0;
            if(input.real < 0) return pi;
            if(input.real == 0) return 0; //arbitrary choice, could be anything
        }
        if(input.real == 0){ //on imaginary axis
            ans = pi / 2 * bMath.sign(input.imag);
        }
        if(input.real > 0){ //in quadrants 1 or 4
            ans = Math.atan(input.imag / input.real);
        }
        if(input.real < 0){ //in quadrants 2 or 3
            ans = Math.atan(input.imag / input.real) + pi * bMath.sign(input.imag);
        }
        return ans;
    }

    /**
     * keeps the direction of a complex number the same while squaring it's magnitude
     * @param input the number you would like to square the magnitude of
     * @return the number on the same line but with it's magnitude squared
     */
    public static ComplexNum squareMagnitude(ComplexNum input){
        ComplexNum ans = input.copy();
        ans.timesEquals(magnitude(input));
        return ans;
    }

    /**
     * gets the distance from 0 + 0i
     * @param input the complex number who's sign you want
     * @return the distance from that number to 0 + 0i
     */
    public static double magnitude(ComplexNum input) {
        return Math.sqrt(magnitudeSqrd(input));
    }

    public static double magnitudeSqrd(ComplexNum input) {
        return input.real * input.real + input.imag * input.imag;
    }

    /**
     * Gets the compliment of the input. The compliment of a complex number is the same thing
     * but with the imaginary component's sign reversed
     * @param input the complex number who's sign you want
     * @return the compliment
     */
    public static ComplexNum compliment(ComplexNum input) {
        ComplexNum ans = new ComplexNum();
        ans.real =  input.real;
        ans.imag = -input.imag;
        return ans;
    }


}