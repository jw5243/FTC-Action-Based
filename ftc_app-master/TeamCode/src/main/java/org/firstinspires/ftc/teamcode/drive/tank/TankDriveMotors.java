package org.firstinspires.ftc.teamcode.drive.tank;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveMotors;

public class TankDriveMotors extends DriveMotors {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public boolean allMotorsPresent() {
        return getLeftMotor() != null && getRightMotor() != null;
    }

    public DcMotor getLeftMotor() {
        return leftMotor;
    }

    public void setLeftMotor(DcMotor leftMotor) {
        this.leftMotor = leftMotor;
    }

    public DcMotor getRightMotor() {
        return rightMotor;
    }

    public void setRightMotor(DcMotor rightMotor) {
        this.rightMotor = rightMotor;
    }
}
