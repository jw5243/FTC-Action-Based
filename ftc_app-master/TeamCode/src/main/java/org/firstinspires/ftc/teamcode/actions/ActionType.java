package org.firstinspires.ftc.teamcode.actions;

public enum ActionType {
    DRIVE_FORWARD, DRIVE_BACKWARD, TURN_LEFT, TURN_RIGHT, STRAFE_LEFT, STRAFE_RIGHT, CUSTOM_ACTION;

    private ActionParameters actionParameters;

    public Action getAction() {
        return (ordinal() == ActionType.DRIVE_FORWARD.ordinal() || ordinal() == ActionType
                .DRIVE_BACKWARD.ordinal() || ordinal() == ActionType.TURN_LEFT.ordinal() ||
                ordinal() == ActionType.TURN_RIGHT.ordinal() || ordinal() == ActionType
                .STRAFE_LEFT.ordinal() || ordinal() == ActionType.STRAFE_RIGHT.ordinal()) &&
                getActionParameters() instanceof DriveActionParameters ? new DriveAction(
                        (DriveActionParameters)(getActionParameters())) : null;
    }

    public ActionParameters getActionParameters() {
        return actionParameters;
    }

    public ActionType setActionParameters(ActionParameters actionParameters) {
        this.actionParameters = actionParameters;
        return this;
    }
}
