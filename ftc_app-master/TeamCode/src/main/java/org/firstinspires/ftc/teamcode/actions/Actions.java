package org.firstinspires.ftc.teamcode.actions;

public class Actions {
    public static void addAction(ActionType actionType) {
        addAction(actionType.getAction());
    }

    public static void addAction(Action action) {
        ActionList.getActionList().add(action);
    }

    public static void sequence(ActionType... actions) {
        Action[] actionsArray = new Action[actions.length];

        for(int index = 0; index < actions.length; index++) {
            actionsArray[index] = actions[index].getAction();
        }

        sequence(actionsArray);
    }

    public static void sequence(Action... actions) {
        for(Action action : actions) {
            addAction(action);
        }
    }

    public static void parallel(int durationMilliseconds, ActionType... actions) {
        Action[] actionsArray = new Action[actions.length];

        for(int index = 0; index < actions.length; index++) {
            actionsArray[index] = actions[index].getAction();
        }

        parallel(durationMilliseconds, actionsArray);
    }

    public static void parallel(int durationMilliseconds, Action... actions) {
        addAction(new Action(durationMilliseconds) {
            @Override
            public void execute() {
                for(Action action : actions) {
                    action.execute();
                }
            }
        });
    }
}
