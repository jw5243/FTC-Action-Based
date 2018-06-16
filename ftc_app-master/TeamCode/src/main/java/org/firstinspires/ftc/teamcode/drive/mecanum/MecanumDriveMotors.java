package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveMotors;

public class MecanumDriveMotors extends DriveMotors {
    private DcMotor backLeftMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;

    public MecanumDriveMotors(DcMotor backLeftMotor, DcMotor frontLeftMotor, DcMotor
            backRightMotor, DcMotor frontRightMotor) {
        setBackLeftMotor(backLeftMotor);
        setFrontLeftMotor(frontLeftMotor);
        setBackRightMotor(backRightMotor);
        setFrontRightMotor(frontRightMotor);
    }

    @Override
    public boolean allMotorsPresent() {
        return getBackLeftMotor() != null && getFrontLeftMotor() != null && getBackRightMotor()
                != null && getFrontRightMotor() != null;
    }

    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }

    public void setBackLeftMotor(DcMotor backLeftMotor) {
        this.backLeftMotor = backLeftMotor;
    }

    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public void setFrontLeftMotor(DcMotor frontLeftMotor) {
        this.frontLeftMotor = frontLeftMotor;
    }

    public DcMotor getBackRightMotor() {
        return backRightMotor;
    }

    public void setBackRightMotor(DcMotor backRightMotor) {
        this.backRightMotor = backRightMotor;
    }

    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    public void setFrontRightMotor(DcMotor frontRightMotor) {
        this.frontRightMotor = frontRightMotor;
    }
}
