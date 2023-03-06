package org.firstinspires.ftc.teamcode.NewSelfDriving;


import java.util.function.DoubleUnaryOperator;


public class Function{
    private DoubleUnaryOperator function;

    /**
     * Constructor to set function. Use a lamba expression:
     * x -> f(x)
     * @param function function that you want. See above on how to create function
     */
    public Function(DoubleUnaryOperator function) {this.function = function;}

    /**
     * Evaluates a given value
     * @param x x value
     * @return the y output
     */
    public double evaluate(double x) {return function.applyAsDouble(x);}

    /**
     * Calculates the derivative
     * @return returns derivative
     */
    public Function derivative(){
        DoubleUnaryOperator derivative = x -> {
            final double dx = 1e-6;
            double y1 = function.applyAsDouble(x);
            double y2 = function.applyAsDouble(x + dx);
            return (y2 - y1) / dx;
        };
        return new Function(derivative);
    }

    /**
     * Changes the function to put in terms of the other variable
     * @return a new function in terms of other variable
     */
    public Function changeVariable() {
        DoubleUnaryOperator newFunction = y -> {
            double lowerBound = -100; // a lower bound for the x values
            double upperBound = 100; // an upper bound for the x values
            final int iterations = 10000; // sample points
            final double decimalError = 1e-6; // the maximum error allowed for approximation

            // Bisection method
            double x = (lowerBound + upperBound) / 2;
            double yPrime = function.applyAsDouble(x);
            for (int i = 0; i < iterations; i++) {
                if (Math.abs(yPrime - y) < decimalError) {
                    return x;
                } else if (yPrime < y) {
                    lowerBound = x;
                } else {
                    upperBound = x;
                }
                x = (lowerBound + upperBound) / 2;
                yPrime = function.applyAsDouble(x);
            }
            // If the inverse function cannot be approximated within the given tolerance, throw an exception
            throw new IllegalArgumentException("Cannot approximate inverse function within tolerance");
        };
        return new Function(newFunction);
    }

    //getter
    Function getFunction(){return new Function(function);}

    //setter
    public void setFunction(DoubleUnaryOperator function) {this.function = function;}
}
