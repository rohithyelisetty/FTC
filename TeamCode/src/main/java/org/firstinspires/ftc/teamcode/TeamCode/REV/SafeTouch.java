package org.firstinspires.ftc.teamcode.TeamCode.REV;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SafeTouch extends SafeHardware{

    private TouchSensor touch;

    public SafeTouch(@NonNull HardwareMap hardwareMap, Telemetry telemetry, String name){
        super(hardwareMap, telemetry, name);

        try {
            touch = hardwareMap.get(TouchSensor.class, name);
        } catch (Exception e) {
            telemetry.addData("Cannot find Touch: ", name);
        }
    }

    public double getValue(){
        return touch.getValue();
    }

    public boolean isPressed(){
        return touch.isPressed();
    }

    public void touchStop(SafeMotor leftMotor, SafeMotor rightMotor,
                          double power, double value){
        double curr_value = touch.getValue();
        while (curr_value < value){
            rightMotor.setPower(power);
            leftMotor.setPower(power);
            curr_value = touch.getValue();
        }
        rightMotor.resetMotor();
        leftMotor.resetMotor();
    }
}
