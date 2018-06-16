package org.firstinspires.ftc.teamcode.drive.tank;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.MotorPowers;

public class TankDriveMotorPowers extends MotorPowers {
    public static final TankDriveMotorPowers FULL_SPEED = new TankDriveMotorPowers(1, 1);
    private double leftMotorPower;
    private double rightMotorPower;

    public TankDriveMotorPowers() {
        this(0, 0);
    }

    public TankDriveMotorPowers(double leftMotorPower, double rightMotorPower) {
        setLeftMotorPower(leftMotorPower);
        setRightMotorPower(rightMotorPower);
    }

    public TankDriveMotorPowers scale(double scaleFactor) {
        return new TankDriveMotorPowers(this.getLeftMotorPower() * scaleFactor, this
                .getRightMotorPower() * scaleFactor);
    }

    public double getLeftMotorPower() {
        return leftMotorPower;
    }

    public void setLeftMotorPower(double leftMotorPower) {
        this.leftMotorPower = Range.clip(Math.abs(leftMotorPower), 0, 1);
    }

    public double getRightMotorPower() {
        return rightMotorPower;
    }

    public void setRightMotorPower(double rightMotorPower) {
        this.rightMotorPower = Range.clip(Math.abs(rightMotorPower), 0, 1);
    }
}
