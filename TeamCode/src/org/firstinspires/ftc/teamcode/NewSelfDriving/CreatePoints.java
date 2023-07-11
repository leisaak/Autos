package org.firstinspires.ftc.teamcode.NewSelfDriving;

import java.util.ArrayList;
import java.util.LinkedList;

public class CreatePoints {
    private static LinkedList<Double> x = new LinkedList<>();
    private  static LinkedList<Double> y = new LinkedList<>();

    public static void addX(Function function, double numPoints, double xLeftBound, double xRightBound){
        double nPoints = ((Math.abs(xLeftBound) + Math.abs(xRightBound))/numPoints);
        for(double i = xLeftBound; i <= xRightBound; i+= nPoints){
            x.add(function.evaluate(i));
        }
    }
    public static void addY(Function function, double numPoints, double yLeftBound, double yRightBound){
        double nPoints = ((Math.abs(yLeftBound) + Math.abs(yRightBound))/numPoints);
        for(double i = yLeftBound; i <= yRightBound; i+= nPoints){
            y.add(function.evaluate(i));
        }
    }

    public static void addCirclePoints(double centerX, double centerY, double radius){
        Function.CircleFunction circle = new Function.CircleFunction(centerX, centerY, radius);
        Function xEquation = new Function(circle.getXEquation());
        Function yEquation = new Function(circle.getYEquation());
        for (double t = 0; t < 2 * Math.PI; t += 0.1) {
            double x = xEquation.evaluate(t);
            double y = yEquation.evaluate(t);
            CreatePoints.x.add(x);
            CreatePoints.y.add(y);
        }
    }

    public static LinkedList<Double> getX() {
        return x;
    }
    public static LinkedList<Double> getY(){
        return  y;
    }
}
