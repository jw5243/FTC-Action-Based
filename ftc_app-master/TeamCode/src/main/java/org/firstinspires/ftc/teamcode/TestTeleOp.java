package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Test")
public class TestTeleOp extends BaseRobot {
    @Override
    public void loop() {
        mecanumDrive();
    }
}