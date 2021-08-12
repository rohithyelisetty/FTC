package org.firstinspires.ftc.teamcode.TeamCode.REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SafeRobot {

    public SafeMotor rightMotor, leftMotor;

    private boolean DEBUG;

    private HardwareMap hwMap;
    private Telemetry telemetry;

    public SafeRobot(){
        DEBUG = false;
    }

    public SafeRobot(boolean debug){
        DEBUG = debug;
    }

    public void initialize(){
        rightMotor = new SafeMotor(hwMap, telemetry, "rightMotor",
                DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor = new SafeMotor(hwMap, telemetry, "leftMotor",
                DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void sendData(String caption, Object value){
        if (!DEBUG) return;
        telemetry.addData(caption, value);
        telemetry.update();
    }
}