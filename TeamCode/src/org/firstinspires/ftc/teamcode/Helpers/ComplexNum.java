package org.firstinspires.ftc.teamcode.Helpers;

//this class is gonna deal with complex numbers, probably will make "Position" class obsolete
public class ComplexNum {
    public double real; //real component
    public double imag; //imaginary component

    public ComplexNum(){
        real = 0;
        imag = 0;
    }
    public ComplexNum(double real, double imag){
        this.real = real;
        this.imag = imag;
    }
    public static ComplexNum newComplexNumPolar(double magnitude, double direction){
        return new ComplexNum(magnitude*Math.cos(direction), magnitude*Math.sin(direction));
    }
    public static ComplexNum add(ComplexNum a, ComplexNum b){
        ComplexNum Ans = new ComplexNum();
        Ans.real = a.real + b.real;
        Ans.imag = a.imag + b.imag;
        return Ans;
    }
    public static ComplexNum subtract(ComplexNum a, ComplexNum b){ //do a - b
        ComplexNum Ans = new ComplexNum();
        Ans.real = a.real - b.real;
        Ans.imag = a.imag - b.imag;
        return Ans;
    }
    public static ComplexNum multiply(ComplexNum a, ComplexNum b){
        ComplexNum Ans = new ComplexNum();
        Ans.real = (a.real * b.real) - (a.imag * b.imag);
        Ans.imag = (a.imag * b.real) + (a.real * b.imag);
        return Ans;
    }

    /**
     * creates a new complex number of the exact same value and rotatoes that about 0 + 0i
     * @param angle angle you want complex number roated by
     * @return the new complex number
     */
    public ComplexNum safeRotateAboutOrigin(double angle){
        ComplexNum Ans = this.copy();
        Ans.rotateAboutOrigin(angle);
        return Ans;
    }
    /**
     * returns Euler's number e raised to the power of a complex number
     * @param input the exponent to raise e to
     * @return the value of e^input, where e is the base of the natural logarithm
     */
    public static ComplexNum exp(ComplexNum input){ //verified
        ComplexNum Ans = bMath.cis(input.imag);
        Ans.timesEquals(Math.exp(input.real));
        return Ans;
    }
    /**
     * @return e raised to the power of this complex number
     */
    public ComplexNum expThis() {
        return exp(this);
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
        double imag = bMath.acis(input);
        double vectorLengthSquared = bMath.sqd(input.imag) + bMath.sqd(input.real);
        double real = 0.5 * Math.log(vectorLengthSquared);
        return new ComplexNum(real, imag);
    }
    public static ComplexNum power(ComplexNum base, ComplexNum exponent){
        return exp(multiply(ln(base), exponent));
    }

    /**
     * rotates this complex number about the point 0 + 0i
     * @param angle angle you want to rotate by (in radians)
     * @return returns itself after rotation
     */
    public ComplexNum rotateAboutOrigin(double angle){
        this.timesEquals(bMath.cis(angle));
        return this;
    }
    /**
     * multiplies this complex number by another complex number
     * @param input the number you would like to multiply by
     * @return
     */
    public ComplexNum timesEquals(ComplexNum input){
        this.equals(Cmath.multiply(this, input));
        return this;
    }
    /**
     * multiplies this by a real number
     * @param input the number you want to multiply by
     * @return this * input
     */
    public ComplexNum timesEquals(double input){
        this.real *= input;
        this.imag *= input;
        return this;
    }

    /**
     * makes a new complex number which is equal to {@code this * i}
     * @return {@code this * i}
     */
    public ComplexNum timesI() {
        return new ComplexNum(-this.imag, this.real);
    }

    public ComplexNum timesEqualsI() {
        this.equals(-imag, real);
        return this;
    }

    /**
     * divides this complex number by another complex number
     * @param input the number you would like to divide by
     * @return this / input
     * @note this will cause a runtime error if you try to divide by 0+0i
     */
    public ComplexNum divideEquals(ComplexNum input) {
        this.divideEquals(Cmath.divide(this, input));
        return this;
    }

    /**
     * divides this by another complex number
     * @param input the number you want to divide by
     * @return this / input
     */
    public ComplexNum divideEquals(double input){
        this.real /= input;
        this.imag /= input;
        return this;
    }

    public ComplexNum plusEquals(ComplexNum input){
        this.equals(add(this,input));
        return this;
    }
    public void plusEquals(double inputReal){
        this.real += inputReal;}
    public ComplexNum minusEquals(ComplexNum input){
        this.equals(subtract(this,input));
        return this;
    }
    public void minusEquals(double input){
        this.real -= input;}
    public ComplexNum squared() {
        this.timesEquals(this);
        return this;
    }

    /**
     * sets a complex number equal to the input complex number
     * @param input value to set this to
     * @return returns itself
     */
    public ComplexNum equals(ComplexNum input){
        real = input.real;
        imag = input.imag;
        return this;
    }
    public ComplexNum equals(double real, double imag){
        this.real = real;
        this.imag = imag;
        return this;
    }

    /**
     * makes a copy with a different memory address
     * @return the copy
     */
    public ComplexNum copy(){
        return new ComplexNum(this.real, this.imag);
    }

}