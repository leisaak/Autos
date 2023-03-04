package org.firstinspires.ftc.teamcode.SelfDriving;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Helpers.Toggle;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

public class SelfDriving extends HardwareHelper {
    static double driveThreshold = .75, turnThreshold = 2.25;
    public static double targetDistance, targetRadian, targetDegrees;
    public boolean turnTableFlag = false, elevatorFlag = false;
    public static double finalError = 0d, timeTook = 0d;
    boolean getOut = false;
    double speed = 0.1;
    public static double first = 0, last = 0;
    public static double firstT, FistCT;
    Toggle move = new Toggle(false);

    /**
     * Movement function for forward and backward
     * @param distance distance in CM for how far to drive
     */
    public void selfDrive(double distance){
        bot.resetOdometers();
        targetDistance = distance;
        bot.resetForPID();
        PID.lastTime = System.nanoTime();
        while(checkDistance() && bot.opmodeIsActive())
        {
            bot.setMotorPower(bot.speedPID(bot.errorDrive(), speed));

            Bot.telemetry.addData("Current speed", bot.speedPID(bot.errorDrive(), speed));
            Bot.telemetry.addData("Current speed", bot.errorDrive());
            Bot.telemetry.addData("Current Position", bot.getLeftOdoPos());
            Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
            Bot.telemetry.addLine("IN SD");
            OpScript.update();
        }
        Bot.telemetry.addLine("Done");
        Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
        OpScript.update();
        bot.setMotorPower(0);
        bot.brake();
    }

    public void selfDrive(double distance, double speed){
        bot.resetOdometers();
        targetDistance = distance;
        bot.resetForPID();
        PID.lastTime = System.nanoTime();
        double startingTime = bot.getTime();
        while(checkDistance() && bot.opmodeIsActive()){
            bot.setMotorPower(Math.copySign(bot.speedPID(bot.errorDrive(), speed), bot.errorDrive()));
            finalError = bot.speedPID(bot.errorDrive(), speed);

            Bot.telemetry.addData("error", bot.errorDrive());
            Bot.telemetry.addData("first pid value", first);
            Bot.telemetry.addData("Current speed", bot.speedPID(bot.errorDrive(), speed));
            Bot.telemetry.addData("Current Position", bot.getLeftOdoPos());
            Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
            OpScript.update();
            last = bot.speedPID(bot.errorDrive(), speed);
        }
        Bot.telemetry.addData("first pid value", first);
        Bot.telemetry.addData("last pid value", last);
        timeTook = bot.getTime() - startingTime;
        Bot.telemetry.addLine("Done");
        Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
        OpScript.update();
        bot.setMotorPower(0);
        bot.brake();
    }

    public void selfDrivePID(double distance, Gamepad gamepad, double speed) {
        bot.resetOdometers();
        targetDistance = distance;
        bot.resetForPID();
        PID.lastTime = System.nanoTime();
        double startingTime = bot.getTime();
        while (checkDistance() && bot.opmodeIsActive() && !getOut) {
            bot.setMotorPower(Math.copySign(bot.speedPID(bot.errorDrive(), speed), bot.errorDrive()));
            finalError = bot.speedPID(bot.errorDrive(), speed);

            if (gamepad.x) {
                getOut = true;
            }
            Bot.telemetry.addData("first pid value", first);
            Bot.telemetry.addData("Current speed", bot.speedPID(bot.errorDrive(), speed));
            Bot.telemetry.addData("Current speed", bot.errorDrive());
            Bot.telemetry.addData("Current Position", bot.getLeftOdoPos());
            Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
            OpScript.update();
            last = bot.speedPID(bot.errorDrive(), speed);
        }
        getOut = false;
        Bot.telemetry.addData("first pid value", first);
        Bot.telemetry.addData("last pid value", last);
        timeTook = bot.getTime() - startingTime;
        Bot.telemetry.addLine("Done");
        Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
        OpScript.update();
        bot.setMotorPower(0);
        bot.brake();
    }

    public void selfDriveLeft(double distance){
        bot.resetOdometers();
        targetDistance = distance;
        bot.resetForPID();
        while(checkDistanceStrafe() && bot.opmodeIsActive()){
            bot.setMotorPowerLeft(bot.speedPID(bot.errorStrafe(), speed));
            Bot.telemetry.addData("Current Position", bot.getMiddleOdoPos());
            OpScript.update();
        }
        Bot.telemetry.addLine("Done");
        OpScript.update();
        bot.setMotorPower(0);
        bot.brake();
    }

    public void selfDriveRight(double distance){
        bot.resetOdometers();
        targetDistance = distance;
        bot.resetForPID();
        while(checkDistanceStrafe() && bot.opmodeIsActive()){
            bot.setMotorPowerRight(bot.speedPID(bot.errorStrafe(), speed));
            Bot.telemetry.addData("Current Position", bot.getMiddleOdoPos());
            OpScript.update();
        }
        Bot.telemetry.addLine("Done");
        OpScript.update();
        bot.setMotorPower(0);
        bot.brake();
    }

    /**
     * Turning function
     * @param degree: degrees of turn
     */
    public void SelfTurn(double degree){
        bot.setPID(.12,0,0);
        bot.resetForPID();
        targetDistance = degree;
        targetRadian = degree;
        while(checkTurn() && bot.opmodeIsActive()){
            bot.setMotorPowerTurn(bot.speedPID(bot.errorTurn(), 0, 0, .07));
            Bot.telemetry.addData("Current Position", bot.angleDEG());
            OpScript.update();
        }
        bot.resetPID(PID.movePID);
        bot.setMotorPower(0);
        bot.brake();
    }

    public void SelfTurn(double degree, double speed){
        targetDistance = Math.toRadians(degree);
        targetRadian = Math.toRadians(degree);
        bot.resetForPID();
        PID.lastTime = System.nanoTime();
        double startingTime = bot.getTime();
//        bot.setPID(PID.turnPID);
        while(checkTurn() && bot.opmodeIsActive()){
            double errorTurn = bot.errorTurn();
            bot.setMotorPowerTurn(Math.copySign(bot.speedPID(errorTurn, speed), errorTurn));
            finalError = bot.speedPID(bot.errorTurn(), speed);

            Bot.telemetry.addData("first pid value", first);
            Bot.telemetry.addData("Current speed", bot.speedPID(bot.errorTurn(), speed));
            Bot.telemetry.addData("Raw IMU angle", bot.angleRAD());
            Bot.telemetry.addData("Current error", bot.errorTurn());
            OpScript.update();
            last = bot.speedPID(bot.errorTurn(), speed);
        }
        Bot.telemetry.addData("first pid value", first);
        Bot.telemetry.addData("last pid value", last);
        timeTook = bot.getTime() - startingTime;
        Bot.telemetry.addLine("Done");
        Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
        OpScript.update();
//        bot.resetPID(PID.movePID);
        bot.setMotorPower(0);
        bot.brake();
    }
    public void selfTurnOneInt(double degree, double speed, Gamepad gamepad){
        targetDistance = Math.toRadians(degree);
        targetRadian = Math.toRadians(degree);
        bot.resetForPID();
        PID.lastTime = System.nanoTime();
        double startingTime = bot.getTime();
//        bot.setPID(PID.turnPID);
            bot.setMotorPowerTurn(Math.copySign(bot.speedPID(bot.errorTurn(), speed), bot.errorTurn()));
            finalError = bot.speedPID(bot.errorTurn(), speed);

            Bot.telemetry.addData("first pid value", first);
            Bot.telemetry.addData("Current speed", bot.speedPID(bot.errorTurn(), speed));
            Bot.telemetry.addData("Current error", bot.errorTurn());
            OpScript.update();
            last = bot.speedPID(bot.errorTurn(), speed);
        Bot.telemetry.addData("first pid value", first);
        Bot.telemetry.addData("last pid value", last);
        timeTook = bot.getTime() - startingTime;
        Bot.telemetry.addLine("Done");
        Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
        OpScript.update();
//        bot.resetPID(PID.movePID);
        bot.setMotorPower(0);
        bot.brake();
    }



    public void selfTurnRelative(double degree, double speed){
        bot.setPID(2.5,.2,.1);
        bot.setPID(PID.turnPID);
        targetDegrees = degree+bot.angleDEG();
         int direction = 1;
        bot.resetForPID();
        if(Math.abs((720-bot.angleDEG()+targetDegrees)%360)<(Math.abs((720-targetDegrees+ bot.angleDEG())%360))) {
            targetDistance = -Math.abs((720-bot.angleDEG()+targetDegrees)%360);
        } else {
            targetDistance = Math.abs((720-targetDegrees+ bot.angleDEG())%360);
        }
        PID.lastTime = System.nanoTime();
        double startingTime = bot.getTime();
        while(checkTurnRelative() && bot.opmodeIsActive()){
            if(Math.abs((720-bot.angleDEG()+targetDegrees)%360)<(Math.abs((720-targetDegrees+ bot.angleDEG())%360))) {
                direction = 1;
            } else {
                direction = -1;
            }
            bot.setMotorPowerTurn(Math.copySign(bot.speedPID(bot.errorTurnDeg(), speed), direction));
            finalError = bot.speedPID(bot.errorTurnDeg(), speed);
            Bot.telemetry.addData("first pid value", first);
            Bot.telemetry.addData("Current speed", bot.speedPID(bot.errorTurnDeg(), speed));
            Bot.telemetry.addData("Raw IMU angle", bot.angleRAD());
            Bot.telemetry.addData("Current error", bot.errorTurn());
            OpScript.update();
            last = bot.speedPID(bot.errorTurnDeg(), speed);
        }
        Bot.telemetry.addData("first pid value", first);
        Bot.telemetry.addData("last pid value", last);
        timeTook = bot.getTime() - startingTime;
        Bot.telemetry.addLine("Done");
        Bot.telemetry.addData("Current Middle Position", bot.getMiddleOdoPos());
        OpScript.update();
        bot.resetPID(PID.movePID);
        bot.setMotorPower(0);
        bot.brake();

    }

    //Booleans which end the turns
    public boolean checkDistance(){return Math.abs(targetDistance - bot.getLeftOdoPos()) > driveThreshold;}
    public boolean checkDistanceStrafe(){return Math.abs(targetDistance) - Math.abs(bot.getMiddleOdoPos()) > driveThreshold;}
    public static boolean checkTurn(){return Math.abs(bMath.subtractAnglesDeg(Math.abs(targetRadian), Math.abs(bot.angleDEG()))) > turnThreshold;}
    public boolean checkTurnRelative() {
        if(Math.abs((720-bot.angleDEG()+targetDegrees)%360)<(Math.abs((720-targetDegrees+ bot.angleDEG())%360))) {
            return Math.abs((720-bot.angleDEG()+targetDegrees)%360) > turnThreshold;
        } else {
            return Math.abs((720-targetDegrees+ bot.angleDEG())%360) > turnThreshold;
        }
    }
}