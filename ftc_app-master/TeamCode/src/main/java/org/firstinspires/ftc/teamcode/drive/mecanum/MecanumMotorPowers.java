package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.MotorPowers;

public class MecanumMotorPowers extends MotorPowers {
    public static final MecanumMotorPowers FULL_SPEED = new MecanumMotorPowers(1, 1, 1, 1);

    private double backLeftMotorPower;
    private double frontLeftMotorPower;
    private double backRightMotorPower;
    private double frontRightMotorPower;

    public MecanumMotorPowers() {
        this(0, 0, 0, 0);
    }

    public MecanumMotorPowers(double backLeftMotorPower, double frontLeftMotorPower, double
            backRightMotorPower, double frontRightMotorPower) {
        setBackLeftMotorPower(backLeftMotorPower);
        setFrontLeftMotorPower(frontLeftMotorPower);
        setBackRightMotorPower(backRightMotorPower);
        setFrontRightMotorPower(frontRightMotorPower);
    }

    public MecanumMotorPowers scale(double scaleFactor) {
        return new MecanumMotorPowers(this.getBackLeftMotorPower() * scaleFactor, this
                .getFrontLeftMotorPower() * scaleFactor, this.getBackRightMotorPower() *
                scaleFactor, this.getFrontRightMotorPower() * scaleFactor);
    }

    public double getBackLeftMotorPower() {
        return backLeftMotorPower;
    }

    public void setBackLeftMotorPower(double backLeftMotorPower) {
        this.backLeftMotorPower = Range.clip(Math.abs(backLeftMotorPower), 0, 1);
    }

    public double getFrontLeftMotorPower() {
        return frontLeftMotorPower;
    }

    public void setFrontLeftMotorPower(double frontLeftMotorPower) {
        this.frontLeftMotorPower = Range.clip(Math.abs(frontLeftMotorPower), 0, 1);
    }

    public double getBackRightMotorPower() {
        return backRightMotorPower;
    }

    public void setBackRightMotorPower(double backRightMotorPower) {
        this.backRightMotorPower = Range.clip(Math.abs(backRightMotorPower), 0, 1);
    }

    public double getFrontRightMotorPower() {
        return frontRightMotorPower;
    }

    public void setFrontRightMotorPower(double frontRightMotorPower) {
        this.frontRightMotorPower = Range.clip(Math.abs(frontRightMotorPower), 0, 1);
    }
}
