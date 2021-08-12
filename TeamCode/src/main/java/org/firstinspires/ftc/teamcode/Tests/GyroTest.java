package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GyroAxisTest")
@Disabled
public class GyroTest extends OpMode {

    ModernRoboticsI2cGyro gyro;

    public void init() {
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.calibrate();
    }

    public void loop() {
        telemetry.addData("Gyro Value", gyro.getIntegratedZValue());
    }
}
