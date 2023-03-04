package org.firstinspires.ftc.teamcode.NewSelfDriving;

/**
 * Abstract class which you should make multiple objects of. These objects are input inside of
 * AutoDrive. These objects are what make the robot move around the field
 */
public abstract class Movement {
    private PIDCoefficients coefficients;//PIDcoef object set in constructor
    private PIDCoefficients pidX, pidY, pidTheta;
    /**
     * Used in AutoDive, this method will be called every iteration in the loop.
     * Put actions in here to run even inside of while loop
     */
    public abstract void runExtra();
    private double dX, dY, dTheta, speed, threshold;//desired values for AutoDrive

    /**
     * Constructor that will tell what AutoDrive to do.
     * ALL VALUES ARE ABSOLUTE. For example, going to 30,30,0 and then calling 30,30,0 again will keep you
     * in the same spot.
     * @param dX desired x absolute position
     * @param dY desired y absolute position
     * @param dTheta desired theta absolute position
     * @param coefficients used to input PIDF coefficients and do PID calculations
     */
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
    //getters
    public PIDCoefficients getCoefficients() {return coefficients;}
    public double getdX(){return dX;}
    public double getdY(){return dY;}
    public double getdTheta(){return dTheta;}
    public double getSpeed(){return speed;}
    public double getThreshold(){return threshold;}
}
