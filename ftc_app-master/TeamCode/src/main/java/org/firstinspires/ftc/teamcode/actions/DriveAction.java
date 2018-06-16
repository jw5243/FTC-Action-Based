package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.drive.MotorPowers;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveMotors;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumMotorPowers;
import org.firstinspires.ftc.teamcode.drive.tank.TankDriveMotorPowers;
import org.firstinspires.ftc.teamcode.drive.tank.TankDriveMotors;

public class DriveAction extends Action {
    private static final double TIME_TO_LENGTH_FACTOR_SLOW_START_STOP = 2d / 3;
    private static final double LENGTH_TO_TIME_FACTOR_SLOW_START_STOP = 3d / 2;

    private DriveMotors driveMotors;
    private MotorPowers maxMotorPowers;
    private Direction direction;
    private double distanceToTravelInches;

    public DriveAction(DriveActionParameters driveActionParameters) {
        this(driveActionParameters.getDriveMotors(), driveActionParameters.getMotorPowers(),
                driveActionParameters.getDirection(), driveActionParameters
                        .getDurationMilliseconds());
    }

    public DriveAction(DriveMotors driveMotors, MotorPowers maxMotorPowers, Direction direction,
            double distanceInches) {
        this(driveMotors, maxMotorPowers, direction, (int)(distanceInches *
                LENGTH_TO_TIME_FACTOR_SLOW_START_STOP * BaseRobot.getDistanceFactor()));
    }

    public DriveAction(DriveMotors driveMotors, MotorPowers maxMotorPowers, Direction direction,
            int durationMilliseconds) {
        setDriveMotors(driveMotors);
        setMaxMotorPowers(maxMotorPowers);
        setDirection(direction);
        setDurationMilliseconds(durationMilliseconds);
        setDistanceToTravelInches((int)(durationMilliseconds *
                TIME_TO_LENGTH_FACTOR_SLOW_START_STOP / BaseRobot.getDistanceFactor()));
        setHasStarted(false);
        setFinished(false);
    }

    @Override
    public void execute() {
        if(getDriveMotors() instanceof TankDriveMotors && getMaxMotorPowers() instanceof TankDriveMotorPowers) {
            TankDriveMotors tankDriveMotors = (TankDriveMotors)(getDriveMotors());
            TankDriveMotorPowers tankDriveMotorPowers = (TankDriveMotorPowers)(calculateSpeeds());
            if(tankDriveMotorPowers != null) {
                int leftMotorDirection = getDirection().ordinal() == Direction.FORWARD.ordinal()
                        || getDirection().ordinal() == Direction.TURN_RIGHT.ordinal() ? 1 : -1;
                int rightMotorDirection = getDirection().ordinal() == Direction.FORWARD.ordinal()
                        || getDirection().ordinal() == Direction.TURN_LEFT.ordinal() ? 1 : -1;
                tankDriveMotors.getLeftMotor().setPower(leftMotorDirection * tankDriveMotorPowers
                        .getLeftMotorPower());
                tankDriveMotors.getRightMotor().setPower(rightMotorDirection *
                        tankDriveMotorPowers.getRightMotorPower());
            }
        } else if(getDriveMotors() instanceof MecanumDriveMotors && getMaxMotorPowers()
                instanceof MecanumMotorPowers) {
            MecanumDriveMotors mecanumDriveMotors = (MecanumDriveMotors)(getDriveMotors());
            MecanumMotorPowers mecanumMotorPowers = (MecanumMotorPowers)(calculateSpeeds());
            if(mecanumMotorPowers != null) {
                int backLeftMotorDirection = getDirection().ordinal() == Direction.FORWARD
                        .ordinal() || getDirection().ordinal() == Direction.TURN_RIGHT.ordinal()
                        || getDirection().ordinal() == Direction.STRAFE_LEFT.ordinal() ? 1 : -1;
                int frontLeftMotorDirection = getDirection().ordinal() == Direction.FORWARD
                        .ordinal() || getDirection().ordinal() == Direction.TURN_RIGHT.ordinal()
                        || getDirection().ordinal() == Direction.STRAFE_RIGHT.ordinal() ? 1 : -1;
                int backRightMotorDirection = getDirection().ordinal() == Direction.FORWARD
                        .ordinal() || getDirection().ordinal() == Direction.TURN_LEFT.ordinal()
                        || getDirection().ordinal() == Direction.STRAFE_RIGHT.ordinal() ? 1 : -1;
                int frontRightMotorDirection = getDirection().ordinal() == Direction.FORWARD
                        .ordinal() || getDirection().ordinal() == Direction.TURN_LEFT.ordinal()
                        || getDirection().ordinal() == Direction.STRAFE_LEFT.ordinal() ? 1 : -1;
                mecanumDriveMotors.getBackLeftMotor().setPower(backLeftMotorDirection *
                        mecanumMotorPowers.getBackLeftMotorPower());
                mecanumDriveMotors.getFrontLeftMotor().setPower(frontLeftMotorDirection *
                        mecanumMotorPowers.getFrontLeftMotorPower());
                mecanumDriveMotors.getBackRightMotor().setPower(backRightMotorDirection *
                        mecanumMotorPowers.getBackRightMotorPower());
                mecanumDriveMotors.getFrontRightMotor().setPower(frontRightMotorDirection *
                        mecanumMotorPowers.getFrontRightMotorPower());
            }
        }
    }

    private MotorPowers calculateSpeeds() throws NullPointerException {
        return getMaxMotorPowers() instanceof TankDriveMotorPowers ? new TankDriveMotorPowers
                (getMotorSpeed(((TankDriveMotorPowers)(getMaxMotorPowers())).getLeftMotorPower())
                        , getMotorSpeed(((TankDriveMotorPowers)(getMaxMotorPowers()))
                        .getRightMotorPower())) : getMaxMotorPowers() instanceof MecanumMotorPowers ? new MecanumMotorPowers(getMotorSpeed(((MecanumMotorPowers)
                (getMaxMotorPowers())).getBackLeftMotorPower()), getMotorSpeed((
                        (MecanumMotorPowers)(getMaxMotorPowers())).getFrontLeftMotorPower()),
                getMotorSpeed(((MecanumMotorPowers)(getMaxMotorPowers())).getBackRightMotorPower
                        ()), getMotorSpeed(((MecanumMotorPowers)(getMaxMotorPowers()))
                .getFrontRightMotorPower())) : null;
    }

    /**
     * This {@code Method} calculates a priori the speed of the motors at the current instant of
     * time and the maximum speed at which the motor (for which this method is being called) is able
     * to run.
     *
     * @param motorPower
     * @return
     */
    private double getMotorSpeed(double motorPower) {
        return (8 * (motorPower * getRuntime().milliseconds() * (3 * BaseRobot.getDistanceFactor
                () * getDistanceToTravelInches() - 2 * (motorPower * getRuntime().milliseconds())
        ) / (9 * Math.pow(BaseRobot.getDistanceFactor() * getDistanceToTravelInches(), 2))));
    }

    public DriveMotors getDriveMotors() {
        return driveMotors;
    }

    public void setDriveMotors(DriveMotors driveMotors) {
        this.driveMotors = driveMotors;
    }

    public MotorPowers getMaxMotorPowers() {
        return maxMotorPowers;
    }

    public void setMaxMotorPowers(MotorPowers maxMotorPowers) {
        this.maxMotorPowers = maxMotorPowers;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public double getDistanceToTravelInches() {
        return distanceToTravelInches;
    }

    public void setDistanceToTravelInches(double distanceToTravelInches) {
        this.distanceToTravelInches = distanceToTravelInches;
    }
}
