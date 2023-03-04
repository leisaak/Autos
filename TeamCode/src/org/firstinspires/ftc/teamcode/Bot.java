package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractInit.Init;
import org.firstinspires.ftc.teamcode.Annotations.Important;
import org.firstinspires.ftc.teamcode.Extra.disabled_samples.EncBot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Features.Timer;
import org.firstinspires.ftc.teamcode.Helpers.Counter;
import org.firstinspires.ftc.teamcode.NewSelfDriving.Movement;
import org.firstinspires.ftc.teamcode.OpModes.OpScript;

/**
 * Singleton class which contains useful methods
 * Bot constructor essentially inits everything
 */
@Important
public class Bot extends Init {
    //Init
    private Bot(OpScript op){super.init(op);}
    public static Bot getInstance(OpScript op) {return new Bot(op);}
    public static Bot getInstance() {return opmode.bot;}
    public boolean opmodeIsActive() {return opmode.opModeIsActive();}
    public boolean isAuto() {return opmode.getClass().isAnnotationPresent(Autonomous.class);}
    public boolean isOpModeRed() {return opmode.getClass().getName().contains("Red");}
    public boolean isOpModeBlue() {return opmode.getClass().getName().contains("Blue");}

    //Motor
    public void setPowers(double LFspeed, double LBspeed, double RFspeed, double RBspeed) {motorData.setPower(LFspeed, LBspeed, RFspeed, RBspeed);}
    public void setMotorPower(double power) {motorData.setMotorPower(power);}
    public void setMotorPowerRight(double power) {motorData.setMotorPowerRight(power);}
    public void setMotorPowerLeft(double power) {motorData.setMotorPowerLeft(power);}
    public void setMotorPowerTurn(double power) {motorData.setMotorPower(-power, power);}
    public void setRunMode(DcMotorEx.RunMode mode) {motorData.setRunMode(mode);}
    public void setPowerBehavior(DcMotorEx.ZeroPowerBehavior Lbehavior, DcMotorEx.ZeroPowerBehavior Rbehavior) {motorData.setPowerBehavior(Lbehavior, Rbehavior);}
    public void brake() {motorData.brake();}
    public void gearShift(Counter counter) {commands.gearShift(opmode.gamepad1, counter);}

    //PID
    public void setPID(double P, double I, double D) {pid.setPID(P, I, D);}
    public void setPID(double[] pids){pid.setPID(pids);}
    public void resetPID(double[] pids) {pid.resetPID(pids);}
    public double speedPID(double error, double speed) {return pid.speed(error, speed);}
    public double speedPID(double error, double min, double max, double speed) {return pid.speed(error, min, max, speed);}
    public double errorDrive() {return pid.errorDrive();}
    public double errorStrafe() {return pid.errorStrafe();}
    public double errorTurn() {return pid.errorTurn();}
    public double errorTurnDeg(){return pid.errorTurnDeg();}
    public void resetForPID() {pid.resetForPID();}

    //Odometers
    public double getLeftOdoPos() {return motorData.getLeftPosition();}
    public double getMiddleOdoPos() {return -motorData.getMiddlePosition();}
    public void resetOdometers() {motorData.resetOdometers();}

    //Angle
    public double angleDEG(){return -imuC.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}
    public double angleRAD(){return -imuC.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
    public void resetIMU() {imuC.resetYaw();}

    //TeleOp
    public void Driver1() {drivers.Driver1(opmode.gamepad1);}
    public void Driver2() {drivers.Driver2(opmode.gamepad2);}
    public void Telemetry() {drivers.telemetry();}
    public void PIDTUNER(){drivers.PIDtuning(opmode.gamepad1);}

    //Movement
    public void move(double distanceCM) {selfDriving.selfDrive(distanceCM, Config.speed);}
    public void move(double distanceCM, double speed) {selfDriving.selfDrive(distanceCM, speed);}
    public void movePIDTune(double distanceCM, double speed){selfDriving.selfDrivePID(distanceCM, opmode.gamepad1, speed);}
    public void moveLeft(double distanceCM) {selfDriving.selfDriveLeft(distanceCM);}
    public void moveRight(double distanceCM) {selfDriving.selfDriveRight(distanceCM);}
    public void turn(double degree) {selfDriving.SelfTurn(degree);}
    public void oldCL(Counter counter){mecanum.coordinateLockExtra(opmode.gamepad1, counter);}
    public void mecanum(Counter counter) {mecanum.mecanumDrive(opmode.gamepad1, counter);}
    public void coordinateLock(Counter counter) {mecanum.coordinateLockMecanum(opmode.gamepad1, counter);}
    public double derivative(double error) {return pid.derivative(error);}
    public void selfTurnPID(double turn, double speed){selfDriving.SelfTurn(turn,speed);}
    public void selfTurnOneIteration(double turn, double speed){selfDriving.selfTurnOneInt(turn,speed,opmode.gamepad1);}
    public void selfTurnRelative(double turn, double speed){selfDriving.selfTurnRelative(turn,speed);}
    public void selfTurn2122(double degree){selfTurning.selfTurn(degree);}
    public void coordinateDrive(Movement movement){autoDrive.drive(movement);}
    public void coordinateDrive(Movement movement, EncBot encBot){autoDrive.drive(movement, encBot);}

    //Features
    public double getTime() {return Timer.getTime();}
    public void resetT() {Timer.reset();}
    public boolean setTimer(double time) {return Timer.timer(time);}
    public void sleepSec(long sec) {Timer.sleep(sec);}
    public void sleepHalfSec(long sec) {Timer.sleepHalfSeconds(sec);}
    public double getVoltage() {return 0d;}
    public void rumble() {commands.rumble(opmode.gamepad1, opmode.gamepad2);}

    //Auto
    public void auto() {commands.run();}
    public void determinePark() {commands.determinePark();}
    public boolean isRunning(){return !getInstance().isAuto() || Bot.runAuto;}
}