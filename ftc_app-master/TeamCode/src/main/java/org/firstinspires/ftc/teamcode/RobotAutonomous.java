package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.Action;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.actions.DriveAction;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumMotorPowers;

@Autonomous
public class RobotAutonomous extends BaseRobot {
    @Override
    public void init() {
        super.init();
        Actions.sequence(new DriveAction(getDriveMotors(), MecanumMotorPowers.FULL_SPEED,
                Direction.FORWARD, 10d), new DriveAction(getDriveMotors(), MecanumMotorPowers
                .FULL_SPEED.scale(0.8), Direction.BACKWARD, 15d), new Action(500) {
            @Override
            public void execute() {
                getArm().setPosition(1);
            }
        }, new DriveAction(getDriveMotors(), MecanumMotorPowers.FULL_SPEED, Direction
                .STRAFE_LEFT, 10d), new DriveAction(getDriveMotors(), MecanumMotorPowers
                .FULL_SPEED, Direction.STRAFE_RIGHT, 10d));
    }

    @Override
    public void loop() {
        runAutonomous();
    }
}