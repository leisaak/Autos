package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Designed to increment a certain variable by a certain amount, based on buttons
 */
public class Counter {
    private double state; //the actual thing you're tracking
    private double buttonLast; //the state of the button during the last cycle
    private long lastChangeCycle;
    double min, max;

    public Counter(double startState){
        state = startState;
        buttonLast = startState;
        lastChangeCycle = 0;
        min = -Double.MAX_VALUE;
        max = Double.MAX_VALUE;
    }

    /**
     * Making a new objects sets these values. Put in the value you want for startState
     * @param startState: sets starting value
     * @param min: minimum value the variable will go to
     * @param max: maximum value the variable will go to
     */
    public Counter(double startState, double min, double max){
        state = startState;
        buttonLast = startState;
        lastChangeCycle = 0;
        this.min = min;
        this.max = max;
    }

    public void clip(double min, double max){
        state = Range.clip(state, min, max);
    }

    /**
     *
     * @param add
     * @param subtract
     */
    public void arithmetic(boolean add, boolean subtract){
        clip(min,max);
        if(add && state == buttonLast){
            buttonLast = state;
            state++;
            lastChangeCycle = Bot.cycleNumber;
        }
        else if(subtract && state == buttonLast){
            buttonLast = state;
            state--;
            lastChangeCycle = Bot.cycleNumber;
        }
        else if (!subtract && !add){
            buttonLast = state;
        }
    }

    /**
     * meant to add, subtract from a total counter based on how much
     * @param add boolean that adds to total
     * @param subtract boolean that subtracts from total
     * @param amount the incrementation based on arithmetic
     */
    public void arithmetic(boolean add, boolean subtract, double amount){
        clip(min, max);
        if(add && state == buttonLast){
            buttonLast = state;
            state += amount;
            lastChangeCycle = Bot.cycleNumber;
        }
        else if(subtract && state == buttonLast){
            buttonLast = state;
            state -= amount;
            lastChangeCycle = Bot.cycleNumber;
        }
        else if (!subtract && !add){
            buttonLast = state;
        }
    }

    public double getNum(){return state;}

    public void setNum(double num){state = num;}

    public boolean justChanged(){
        if(Bot.cycleNumber > 2) {
            return Bot.cycleNumber == lastChangeCycle || Bot.cycleNumber - 1 == lastChangeCycle;
        }
        else{
            return false;
        }
    }
}
