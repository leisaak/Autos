package org.firstinspires.ftc.teamcode.NewSelfDriving;

public class PIDCoefficients {
    private double P, I, D, F;
    private double errorSum, dError, lastError, lastTime;
    private double kP, kI, kD, kF;
    public PIDCoefficients(double P, double I, double D, double F){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
    }
    public double getP(){return P;}
    public double getI(){return I;}
    public double getD(){return D;}
    public double getF(){return F;}

    public double getPID(double error, double target, double speed){
        if(target == 0) {target += 0.01;}
            double currentTime = System.nanoTime();
            //pid calculation
            errorSum += (error / target) * ((currentTime - lastTime) / 1000000000); // Update integral accumulator
            dError = (((currentTime - lastTime) / 1000000000) > .001) ? ((error / target) - lastError) / ((currentTime - lastTime) / 1000000000) : 0;
            lastError = error / target; // Update previous error
            lastTime = currentTime;

            kP = P * (error / target);
            kI = I * errorSum;
            kD = D * dError;
            kF = 0d;
            return (kP + kI + kD) * speed;
    }
    public double getPID(PIDCoefficients pidCoefficients, double error, double target, double speed){
        if(target == 0) {target += 0.01;}
            double currentTime = System.nanoTime();
            //pid calculation
            errorSum += (error / target) * ((currentTime - lastTime) / 1000000000); // Update integral accumulator
            dError = (((currentTime - lastTime) / 1000000000) > .001) ? ((error / target) - lastError) / ((currentTime - lastTime) / 1000000000) : 0;
            lastError = error / target; // Update previous error
            lastTime = currentTime;

            kP = pidCoefficients.getP() * (error / target);
            kI = pidCoefficients.getI() * errorSum;
            kD = pidCoefficients.getD() * dError;
            kF = 0d;
            return (kP + kI + kD) * speed;
    }

    public void resetForPID(){
        dError = 0d;
        errorSum = 0d;
    }

    public double getkP(){return kP;}
    public double getkI(){return kI;}
    public double getkD(){return kD;}
    public double getkF(){return kF;}
}
