package org.firstinspires.ftc.teamcode.actions;

import java.util.ArrayList;
import java.util.List;

public class ActionList {
    private static List<Action> actionList = new ArrayList<>();

    public static List<Action> getActionList() {
        return actionList;
    }

    public static void execute() {
        if(getActionList().size() > 0) {
            if(getActionList().get(0).isFinished()) {
                getActionList().remove(0);
            }

            getActionList().get(0).start();
            getActionList().get(0).execute();
        }
    }
}
