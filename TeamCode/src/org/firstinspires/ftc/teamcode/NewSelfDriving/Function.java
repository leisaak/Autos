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

    public static double curveDistance(PathBuilder path){
        double sum = 0d;
        for(int i = 1; i < path.getPath().size(); i++){
            Movement p1 = path.getPath().get(i), p2 = path.getPath().get(i-1);
            //two coordinate points
            double xC = CreatePoints.getX().get(i), xP = CreatePoints.getX().get(i-1);
            double yC = CreatePoints.getY().get(i), yP = CreatePoints.getY().get(i-1);

            //calculating hypotenuse of points
            sum += Math.sqrt(Math.pow(xC - xP, 2) + Math.pow(yC - yP, 2));
        }
        return sum;
    }

    public static double hypotenuse(double a, double b){
        return Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
    }

    //setter
    public void setFunction(DoubleUnaryOperator function) {this.function = function;}

    public static class CircleFunction{
        private final double centerX, centerY,radius;

        public CircleFunction(double centerX, double centerY, double radius) {
            this.centerX = centerX;
            this.centerY = centerY;
            this.radius = radius;
        }

        public DoubleUnaryOperator getXEquation() {
            return x -> centerX + radius * Math.cos(x);
        }

        public DoubleUnaryOperator getYEquation() {
            return y -> centerY + radius * Math.sin(y);
        }
    }
}
