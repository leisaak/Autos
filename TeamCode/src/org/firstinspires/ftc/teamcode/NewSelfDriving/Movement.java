package org.firstinspires.ftc.teamcode.NewSelfDriving;

public abstract class Movement {
    private PIDCoefficients coefficients;
    public abstract void runExtra();
    private double dX, dY, dTheta, speed, threshold;

    public Movement(double dX, double dY, double dTheta, PIDCoefficients coefficients){
        this.dX = dX;
        this.dY = dY;
        this.dTheta = dTheta;
        speed = 0.5;
        this.coefficients = coefficients;
    }
    public Movement(double dX, double dY, double dTheta, double speed, double threshold){
        this.dX = dX;
        this.dY = dY;
        this.dTheta = dTheta;
        this.speed = speed;
        this.threshold = threshold;
    }
    public double getdX(){return dX;}
    public double getdY(){return dY;}
    public double getdTheta(){return dTheta;}
    public double getSpeed(){return speed;}
    public double getThreshold(){return threshold;}

    public PIDCoefficients getCoefficients() {return coefficients;}
}
