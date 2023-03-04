package org.firstinspires.ftc.teamcode.NewSelfDriving;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Extra.disabled_samples.EncBot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

public class AutoDrive extends HardwareHelper {
    double currentX, currentY, currentTheta;
    double driveThreshold = 1, turnThreshold = 0.25;
    double lastX, lastY, saveX, saveY;

    public AutoDrive(){
    }

    /**
     * This method moves the robot to a desired x,y coordinate with a wanted theta
     * @param movement object that inputs x,y,theta, coefficients, etc. Used to run and calculate
     */
    public void drive(Movement movement){
        movement.getCoefficients().resetForPID();
        currentTheta = bMath.subtractAnglesDeg(movement.getdTheta(), bot.angleDEG());
        while(xCondition(movement) || thetaCondition()){
            lastX = bot.getMiddleOdoPos();
            lastY = bot.getLeftOdoPos();
            double xPID, yPID, rxPID;
            double dX = movement.getdX(), dY = movement.getdY(), dTheta = movement.getdTheta();

            currentX = bot.getMiddleOdoPos() * Math.cos(bot.angleDEG()) + bot.getLeftOdoPos() * Math.sin(bot.angleDEG());
            currentY = bot.getMiddleOdoPos() * Math.sin(bot.angleDEG()) + bot.getLeftOdoPos() * Math.cos(bot.angleDEG());
            currentTheta = bMath.subtractAnglesDeg(dTheta, bot.angleDEG());

            xPID = movement.getCoefficients().getPID(dX - currentX, dX, Config.speed);
            yPID = movement.getCoefficients().getPID(dY - currentY, dY, Config.speed);
            rxPID = movement.getCoefficients().getPID(currentTheta, dTheta, 0.5);

            movement.runExtra();
            driving(xPID, yPID, rxPID);

            Bot.telemetry.addData("y", currentY);
            Bot.telemetry.addData("x", currentX);
            Bot.telemetry.addData("theta", currentTheta);
            Bot.telemetry.addData("ypid", yPID);
            Bot.telemetry.addData("xpid", xPID);
            Bot.telemetry.addData("thetapid", rxPID);
            Bot.telemetry.addData("deg:", bot.angleDEG());
            OpScript.update();
        }
    }
    double[] pose = new double[3];
    public void drive(Movement movement, EncBot odo){
        odo.in();
        movement.getCoefficients().resetForPID();
        currentTheta = bMath.subtractAnglesDeg(movement.getdTheta(), bot.angleDEG());
        while(xCondition(movement)){
            pose = odo.updateOdometry();
            double xPID, yPID, rxPID;
            double dX = movement.getdX(), dY = movement.getdY(), dTheta = movement.getdTheta();

            currentTheta = bMath.subtractAnglesDeg(dTheta, bot.angleDEG());

            xPID = movement.getCoefficients().getPID(dX - pose[0], dX, Config.speed);
            yPID = movement.getCoefficients().getPID(dY - pose[1], dY, Config.speed);
            rxPID = movement.getCoefficients().getPID(currentTheta, dTheta, 0.5);

            movement.runExtra();
            driving(xPID, yPID, rxPID);

            Bot.telemetry.addData("y", pose[1]);
            Bot.telemetry.addData("x", pose[0]);
            Bot.telemetry.addData("theta", currentTheta);
            Bot.telemetry.addData("ypid", yPID);
            Bot.telemetry.addData("xpid", xPID);
            Bot.telemetry.addData("thetapid", rxPID);
            Bot.telemetry.addData("deg:", bot.angleDEG());
            OpScript.update();
        }
    }
    //drive there
    private void driving(double x, double y, double rx){
    double rotX = x * Math.cos(bot.angleRAD()) - y * Math.sin(bot.angleRAD());
    double rotY = x * Math.sin(bot.angleRAD()) + y * Math.cos(bot.angleRAD());
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;

        bot.setPowers(frontLeftPower,
            backLeftPower,
            frontRightPower,
            backRightPower);
    }

    //boolean conditions which check loop to see if we made it
    boolean xCondition(Movement movement){return Math.abs(movement.getdX() - currentX) > driveThreshold;}
    boolean yCondition(Movement movement){return Math.abs(movement.getdY() - currentY) > driveThreshold;}
    boolean thetaCondition(){return Math.abs(currentTheta) > turnThreshold;}
    boolean movementCondition(Movement movement){return  xCondition(movement) || yCondition(movement) || thetaCondition();}
}
