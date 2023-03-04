package org.firstinspires.ftc.teamcode.Helpers;//package org.firstinspires.ftc.teamcode.Helpers;


import org.firstinspires.ftc.teamcode.Bot;

/**
 * This class is basically a wrapper class for booleans, but it allows you to negate
 * the boolean by pressing a button. The intended use case is that there is some boolean
 * value used in an OpMode, and you want it to change every time you press a specific button
 * on the gamepad. Make that boolean a {@code Toggle} and call {@code toggle(boolean)}
 * every cycle using the desired button as the input to {@code toggle()}
 */
public class Toggle { //use this class when you want to have a boolean you toggle around
    private boolean state; //the actual thing you're tracking
    private boolean buttonLast; //the state of the button during the last cycle
    //used to know if the state changed from false to true last cycle
    private boolean justChangedFlag;
    private long lastChangeCycle;

    public Toggle(boolean startState){
        state = startState;
        buttonLast = false;
        lastChangeCycle = 0;
    }

    /**
     * if the input to this method is true, and the last time this method was
     * called the input was false, then the boolean this object tracks is {@code state = !state}
     * @param button
     */
    public void toggle(boolean button){
        justChangedFlag = false;
        if(button && !buttonLast){
            state = !state;
            lastChangeCycle = Bot.cycleNumber;
            justChangedFlag = true;
        }
        buttonLast = button;
    }

    /**
     * sets the value of {@code state} on this object
     * @param bool the value you want {@code state} to have
     */
    public void set(boolean bool){
        if(bool != state){
            state = bool;
            lastChangeCycle = Bot.cycleNumber;
            justChangedFlag = true;
        }
        else{ //this means state is already set to what it should be
            justChangedFlag = false;
        }
    }

    /**
     * gets the {@code state} value of this {@code toggle} object
     * @return {@code this.state}
     */
    public boolean getBool(){
        return state;
    }

    /**
     * used to know if the state changed the most recent time state was assigned ( either by
     * {@code toggle()} or {@code set()} )
     * @return {@code true} if it did; {@code false} if it didn't.
     */
    public boolean justChanged(){
        if(Bot.cycleNumber > 2) {
            return Bot.cycleNumber == lastChangeCycle || Bot.cycleNumber - 1 == lastChangeCycle;
        }
        else{
            return false;
        }
    }

    /**
     * used to know if the state changed from {@code false} to {@code true} the most recent time
     * state was assigned ( either by {@code toggle()} or {@code set()} )
     * @return {@code true} if it did; {@code false} if it didn't.
     */

    public boolean justBecameTrue(){
        return (justChanged() && state);
    }

    /**
     * tells you if the state changed from {@code true} to {@code false} the most recent time
     * state was assigned ( either by {@code toggle()} or {@code set()} )
     * @return {@code true} if it did; {@code false} if it didn't.
     */
    public boolean justBecameFalse(){
        return (justChanged() && !state);
    }

    /**
     * never call this method from anywhere
     */
    private void justLookingAtTheJavaDocs(){ //literally just want to read the java docs of all these methods
        justChanged();
        justBecameTrue();
        justBecameFalse();
        set(true);
    }
}
