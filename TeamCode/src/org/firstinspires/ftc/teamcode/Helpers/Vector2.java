package org.firstinspires.ftc.teamcode.Helpers;

// Two dimensional double vector
public class Vector2 {
    public static final Vector2 zero = new Vector2(0, 0,0);
    public double setXs;
    public double setYs;
    public double setTheta;
    protected double x;
    protected double y;
    protected double theta;

    // Creates the vector (0,0)
    public Vector2() {
        x = 0;
        y = 0;
    }
    public Vector2(double xValue, double yValue, double thetaValue) {
        x = xValue;
        y = yValue;
        theta = thetaValue;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta(){
        return theta;
    }
    public void setX(double value){
        x = value;
    }
    public void setY(double value){
        y = value;
    }
    public void sexXandY(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setXandYandTheta(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

     //Adds the x and y values of vector A and B and returns the resulting vector
    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y,0);
    }

    // Subtracts the x and y values of vector A and B and returns the resulting vector
//    public static Vector2 subtract(Vector2 a, Vector2 b) {
//        return new Vector2(a.x - b.x, a.y - b.y);
//    }

    // Divides the x and y values of vector A and B and returns the resulting vector
//    public static Vector2 divide(Vector2 a, Vector2 b) {
//        return new Vector2(a.x / b.x, a.y / b.y);
//    }

    // Divides the x and y values of vector A by double B and returns the resulting vector
//    public static Vector2 divide(Vector2 a, double b) {
//        return new Vector2(a.x / b, a.y / b);
//    }

//    public static Vector2 normalOf(Vector2 value) {
//        double magnitude = magnitude(value);
//
//        if (magnitude(value) > 0.0001) {
//            return divide(value, magnitude);
//        } else {
//            return zero;
//        }
//    }

    // Returns the dot produce of a and b, between -1 and 1
    public static double dot(Vector2 a, Vector2 b) {
        return (a.x * b.x) + (a.y * b.y);
    }

    // Returns the magnitude of this vector
    public static double magnitude(Vector2 value) {
        return Math.sqrt((value.x * value.x) + (value.y * value.y));
    }

    // Returns the squared magnitude of this vector, negligibly faster
    public static double magnitudeSq(Vector2 value) {
        return (value.x * value.x) + (value.y * value.y);
    }

    public static double Angle(Vector2 a, Vector2 b){
        return Math.acos((Vector2.dot(a,b))/(Vector2.magnitude(a) * Vector2.magnitude(b)));

    }    // Returns a heading vector from a radian angle measure
//    public static Vector2 fromRadians(double angle) {
//        return new Vector2(Math.cos(angle), Math.sin(angle));
//    }

}
