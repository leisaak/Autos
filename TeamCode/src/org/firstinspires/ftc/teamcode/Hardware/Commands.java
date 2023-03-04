package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AbstractInit.HardwareHelper;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Features.Config;
import org.firstinspires.ftc.teamcode.Helpers.Counter;

/**
 * Contains auto methods for both sides, methods to move elevator, turn table
 */
public class Commands extends HardwareHelper {
    public double square(double square){return Config.squares(square);}
    int elevatorConeStack = Config.elevatorConeStack;
    int turnTableFirst = 0;
    int crazyTurnTable1 = 0, crazyTurnTable2 = 0, crazyTurnTable3 = 0;
    double turn = 0d;

    public int turnTableRight(){return Config.turnTableRight90;}
    public int turnTableLeft(){return Config.turnTableLeft90;}
    public int resetTurnTable(){return -42;}

    public void gearShift(Gamepad gamepad, Counter counter){counter.arithmetic(gamepad.right_bumper, gamepad.left_bumper, .1);}


    public void rumble(Gamepad gamepad, Gamepad gamepad2){
        gamepad.rumble(3000);
        gamepad2.rumble(3000);
    }

    //1 is far left, 2 is middle, 3 is right
    public void determinePark(){
    }

    /**
     * Sets the variables so that our auto matches both sides
     */
    public void setTurnTableAuto(){
        if(bot.isOpModeRed()){
            turnTableFirst = turnTableRight();
            turn = -90;

            crazyTurnTable1 = turnTableLeft();
            crazyTurnTable2 = turnTableLeft() + turnTableLeft()/2;
            crazyTurnTable3 = turnTableRight() + turnTableRight()/2;
        }
        else if(bot.isOpModeBlue()){
            turnTableFirst = turnTableLeft();

            crazyTurnTable1 = turnTableRight();
            crazyTurnTable2 = turnTableRight() + turnTableRight()/2;
            crazyTurnTable3 = turnTableLeft() + turnTableLeft()/2;
            turn = 90;
        }
    }

    /**
     * Use this auto for all autos. Sets it based off name of class
     */
    public void run(){

    }
    public void Park(){
        Config.speed = 0.5;
        bot.move(square(1));
        bot.turn(-90);
        if(Bot.barcode == 1){
         bot.move(square(1));
        }
        else if(Bot.barcode == 2){
            bot.move(square(-1));
        }

    }

    public void pickUpConeStack(){
        //for picking up a cone
        elevatorConeStack += 100;
    }
    public void coneStackCounter(Counter counter){
        if(counter.getNum() == 1){elevatorConeStack = Config.elevatorConeStack + 400;}
        else if(counter.getNum() == 2){elevatorConeStack = Config.elevatorConeStack + 300;}
        else if(counter.getNum() == 3){elevatorConeStack = Config.elevatorConeStack + 200;}
        else if(counter.getNum() == 4){elevatorConeStack = Config.elevatorConeStack + 100;}
        else if(counter.getNum() == 5){elevatorConeStack = Config.elevatorConeStack;}
    }
}
