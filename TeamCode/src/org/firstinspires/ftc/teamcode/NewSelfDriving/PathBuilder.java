package org.firstinspires.ftc.teamcode.NewSelfDriving;

import java.util.LinkedList;

public class PathBuilder {
    private LinkedList<Movement> path;

    public PathBuilder(LinkedList<Movement> path){
        this.path = path;
    }
    public static LinkedList<Movement> createPath(LinkedList<Double> xPoints, LinkedList<Double> yPoints, double dTheta, PIDCoefficients pid, Extra extra){
        LinkedList<Movement> path = new LinkedList<>();
        //size out any extra points
        if(xPoints.size() > yPoints.size()){
            for(int i = yPoints.size(); i < xPoints.size(); i++){xPoints.remove(i);}
        } else if(yPoints.size() > xPoints.size()){
            for(int i = xPoints.size(); i < yPoints.size(); i++){yPoints.remove(i);}
        }
        //create movement objects
        for(int i = 0; i < xPoints.size(); i++){
            path.add(new Movement(xPoints.get(i), yPoints.get(i), dTheta,pid) {
                @Override
                public void runExtra() {
                    extra.runExtra();
                }
            });
        }
        return path;
    }

    public LinkedList<Movement> getPath(){return path;}

    public interface Extra{
        void runExtra();
    }
}
