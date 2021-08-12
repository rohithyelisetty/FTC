package org.firstinspires.ftc.teamcode.TeamCode.REV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous")
@Disabled
public class SafeAutonomous extends LinearOpMode {

    private SafeRobot robot;

    public void runOpMode() {

        robot = new SafeRobot(true);
        robot.initialize();

        waitForStart();

//        robot.colorDistanceSensor.colorStop(robot.leftMotor, robot.rightMotor,
//                32, 83, 198, 50, 0.1);
//        robot.imu.imuTurn(robot.leftMotor, robot.rightMotor, 90, 2);
//        robot.imu.imuStraight(robot.leftMotor, robot.rightMotor,
//                90, 0.5, 4);
//        robot.imu.imuTurn(robot.leftMotor, robot.rightMotor, 0, 2);
//        robot.colorDistanceSensor.distanceStop(robot.leftMotor, robot.rightMotor,
//                12, 0.5, 1);
//        robot.touchSensor.touchStop(robot.leftMotor, robot.rightMotor, 0.5, 1);
    }
}