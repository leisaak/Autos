package org.firstinspires.ftc.teamcode.Helpers;

import java.util.ArrayList;

/**
 * going to have generally useful methods
 */
public class Utils {

    /**
     * sets all the elements of a boolean array to false
     * @param array the array you want set to false
     * @return the same array you inputed, but mutated to everything is false
     */
    public static boolean[] setBooleanArrayToFalse(boolean[] array){
        for (int i = 0; i < array.length; i++){
            array[i] = false;
        }
        return array;
    }

    //TODO Generalize for all types
    /**
     * sets an array to a specific value
     */
    public static double[] fillArray(double[] array, double value) {
        for (int i = 0; i < array.length; i++) {
            array[i] = value;
        }
        return array;
    }

    public static double getMAD(ArrayList<Double> list){ //MAD = mean absolute deviation
        double total = 0;
        int count = list.size();
        for(int i = 0; i < count; i++){
            total += list.get(i);
        }
        double mean = total / count;
        double totalDeviation = 0;
        for(int i = 0; i < count; i++){
            totalDeviation += Math.abs(mean - list.get(i));
        }
        return totalDeviation / count;
    }
}
