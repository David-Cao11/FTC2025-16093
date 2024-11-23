package org.firstinspires.ftc.teamcode.uppersystems;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Action {
    private int error;
    private SuperStructure upper;
    public final static ArrayList<Action> actions = new ArrayList<>(6);

    public int getError(){
        return 0;
    }

    public void actuate(){

    }

    public boolean isFinished(){
        return false;
    }

    public boolean canStartNext(){
        return false;
    }


    public String returnType() {
        return "Action";
    }

    public void forceStop(){
    }

    public static void buildSequence(Runnable runWhileBuilding){
        if(!actions.isEmpty()){
            for (int i=0;i < actions.size();i++) {
                Action currentAction = actions.get(i);
                currentAction.actuate(); // Execute current action

                while(!currentAction.canStartNext()){
                    runWhileBuilding.run();

                    if(currentAction.isFinished()){
                        currentAction.forceStop();
                        break;
                    }
                }
            }
            actions.clear(); // Clear completed actions and reset mode
        }
    }
}
