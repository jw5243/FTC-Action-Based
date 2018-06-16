package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.drive.MotorPowers;

public class DriveActionParameters extends ActionParameters {
    private DriveMotors driveMotors;
    private MotorPowers motorPowers;
    private Direction direction;
    private int durationMilliseconds;

    public DriveActionParameters(DriveMotors driveMotors, MotorPowers motorPowers, int
            durationMilliseconds) {
        this(driveMotors, motorPowers, Direction.FORWARD, durationMilliseconds);
    }

    public DriveActionParameters(DriveMotors driveMotors, MotorPowers motorPowers, Direction
            direction, int durationMilliseconds) {
        setDriveMotors(driveMotors);
        setMotorPowers(motorPowers);
        setDirection(direction);
        setDurationMilliseconds(durationMilliseconds);
    }

    public DriveMotors getDriveMotors() {
        return driveMotors;
    }

    public void setDriveMotors(DriveMotors driveMotors) {
        this.driveMotors = driveMotors;
    }

    public MotorPowers getMotorPowers() {
        return motorPowers;
    }

    public void setMotorPowers(MotorPowers motorPowers) {
        this.motorPowers = motorPowers;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getDurationMilliseconds() {
        return durationMilliseconds;
    }

    public void setDurationMilliseconds(int durationMilliseconds) {
        this.durationMilliseconds = durationMilliseconds;
    }
}
