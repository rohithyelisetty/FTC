package org.firstinspires.ftc.teamcode.TeamCode.REV;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp")
public class SafeTeleOp extends OpMode {

    private SafeRobot robot;

    public void init() {
        robot = new SafeRobot(true);
        robot.initialize();
    }

    public void loop() {
        SafeMotor.joystickDrive(gamepad1, robot.leftMotor, robot.rightMotor);
    }
}