package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Annotations.Driver;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Helpers.Counter;
import org.firstinspires.ftc.teamcode.Helpers.Toggle;
import org.firstinspires.ftc.teamcode.Helpers.bMath;
import org.firstinspires.ftc.teamcode.SelfDriving.PID;
import org.firstinspires.ftc.teamcode.SelfDriving.SelfDriving;

/**
 * This is a Java code for a FTC (FIRST Tech Challenge) robot driver control. The code is a class called "Drivers" that extends a class called "HardwareHelper".
 *
 * The class contains several instance variables, including two Toggle objects (servoToggle and mecanumToggle), a Counter object (turnTable), and a Counter object (scalePower). The "Driver1" and "Driver2" methods take a Gamepad object as input and use the buttons on the gamepad to control various functions of the robot. The telemetry method outputs telemetry data such as the scale power, heading, and other values.
 *
 * The "Driver1" method toggles the mecanum drive between regular mecanum drive and coordinate lock mecanum drive. The method also resets the IMU (inertial measurement unit) if the "x" button on the gamepad is pressed.
 *
 * The "Driver2" method has several functions including calibrating the arm, controlling the turn table position, and controlling the elevator height. The method uses the buttons on the gamepad to perform these functions. The turnTable and ConeStack Counter objects are used to control the turn table position and elevator height respectively. The servoToggle toggle object is used to control the grabber servo. The method also contains conditions to control the elevator height based on the button pressed.
 *
 * The telemetry method outputs telemetry data such as the scale power, heading, and other values.
 */
public class Drivers extends HardwareHelper {
    private final Toggle servoToggle = new Toggle(false);
    private final Toggle mecanumToggle = new Toggle(true);
    private final Toggle calibration = new Toggle(false);

    private final Counter turnTable = new Counter(2, 1, 3);
    private final Counter scalePower = new Counter(.3, .2, 0.8);
    private final Counter ConeStack = new Counter(3, 1, 5);
    private final Counter Speed = new Counter(.7);


    //PID tuning
    //driving 1,.16,.2
    //turning 1,0,0
    Counter[] movePID = {new Counter(1,0,5), new Counter(0.3,0,5), new Counter(0.2,0,5)};
    Counter[] turnPID = {new Counter(1,0,5), new Counter(0.1,0,5), new Counter(0.16,0,5)};
    private final Counter P = new Counter(1, 0, 5);
    private final Counter I = new Counter(0, 0, 5);
    private final Counter D = new Counter(0, 0, 5);
    private final Counter DISTANCE = new Counter(90, 0, 150);
    private final Toggle ppOne = new Toggle(false);
    private final Toggle pppOne = new Toggle(false);
    private final Toggle move = new Toggle(false);
    double amount = 0.1;


    @Driver.Driver1.Seth
    public void Driver1(Gamepad gamepad) {
        mecanumToggle.toggle(gamepad.y);//toggles between regular and coordinate lock mecanum

        if(mecanumToggle.getBool()){bot.mecanum(scalePower);}
        else{bot.oldCL(scalePower);}

        if(gamepad.right_bumper){bot.resetIMU();}//resets IMU just in case robot is not facing directly north
    }

    @Driver.Driver2.Lil_Hendry
    public void Driver2(Gamepad gamepad){
    }

    @Driver.Coach.Christian
    public void telemetry(){
        Bot.telemetry.addData("x: ", bot.getMiddleOdoPos() * Math.cos(bot.angleDEG()) + bot.getLeftOdoPos() * Math.sin(bot.angleDEG()));
        Bot.telemetry.addData("y: ", bot.getMiddleOdoPos() * Math.sin(bot.angleDEG()) + bot.getLeftOdoPos() * Math.cos(bot.angleDEG()));
        Bot.telemetry.addData("theta: ", bMath.subtractAnglesDeg(90, bot.angleDEG()));

        Bot.telemetry.addData("Heading: ", bot.angleDEG());

        Bot.telemetry.addLine();


        Bot.telemetry.addData("Scale Power", scalePower.getNum());
        Bot.telemetry.addData("Standard Mecanum", mecanumToggle.getBool());
        Bot.telemetry.addData("Turn Table Position", turnTable.getNum());
        Bot.telemetry.addData("Cone stack height", ConeStack.getNum());
    }

    public void PIDtuning(Gamepad gamepad){
        //update PID
        P.arithmetic(gamepad.dpad_up, gamepad.dpad_down, amount);
        I.arithmetic(gamepad.dpad_right, gamepad.dpad_left, amount);
        D.arithmetic(gamepad.right_bumper, gamepad.left_bumper, amount);
        Speed.arithmetic(gamepad.left_stick_y < -Config.DEADZONE, gamepad.left_stick_y > Config.DEADZONE, amount);
        DISTANCE.arithmetic(gamepad.right_trigger > Config.DEADZONE, gamepad.left_trigger > Config.DEADZONE, 5);

        //set toggles
        ppOne.toggle(gamepad.y);
        pppOne.toggle(gamepad.x);
        move.toggle(gamepad.a);

        //change amounts based on toggles
        if (ppOne.getBool() && !pppOne.getBool()){amount = 0.1;}
        else if(pppOne.getBool() && !ppOne.getBool()){amount = 0.01;}
        else if(pppOne.getBool() && ppOne.getBool()){amount = 1;}
        else{amount = 0.001;}

        //set PID
        bot.setPID(P.getNum(), I.getNum(), D.getNum());

        //move it
        if(gamepad.a) {
            bot.resetT();
            bot.selfTurn2122(DISTANCE.getNum());
        } else if(gamepad.b){
            bot.resetT();
            bot.selfTurn2122(-DISTANCE.getNum());
        }

        //Telemetry
        Bot.telemetry.addData("IMU", bot.angleDEG());
        Bot.telemetry.addData("Proportional: ", "%.5f", P.getNum());
        Bot.telemetry.addData("Integral: ",  "%.5f", I.getNum());
        Bot.telemetry.addData("Derivative: ",  "%.5f", D.getNum());
        Bot.telemetry.addData("Distance: ", DISTANCE.getNum());

        Bot.telemetry.addLine();

        Bot.telemetry.addData("1^-1: ", ppOne.getBool());
        Bot.telemetry.addData("1^-2: ", pppOne.getBool());
        Bot.telemetry.addData("Amount: ", amount);

        Bot.telemetry.addLine();

        Bot.telemetry.addData("Final error: ", SelfDriving.finalError);
        Bot.telemetry.addData("Time: ",  SelfDriving.timeTook);

        Bot.telemetry.addLine();

        Bot.telemetry.addData("Sped: ", Speed.getNum());

        Bot.telemetry.addData("first pid value", SelfDriving.first);
        Bot.telemetry.addData("last pid value", SelfDriving.last);
        Bot.telemetry.addData("first time delta", SelfDriving.firstT);
        Bot.telemetry.addData("First D value", SelfDriving.FistCT);

        Bot.telemetry.addData("Int time diff", PID.timeDifference);
        Bot.telemetry.addData("total time differnece", PID.timeSum / 1000000000);
    }
}