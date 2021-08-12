package org.firstinspires.ftc.teamcode.TeamCode.REV;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SafeServo extends SafeHardware{

    private Servo servo;

    public enum SafeServoLadderDirections {
        UP,
        DOWN
    }

    public SafeServo(@NonNull HardwareMap hardwareMap, Telemetry telemetry, String name,
                     Servo.Direction direction) {
        super(hardwareMap, telemetry, name);

        try {
            servo = hardwareMap.get(Servo.class, name);
            servo.setDirection(direction);
        } catch (Exception e) {
            telemetry.addData("Cannot find Servo: ", name);
        }
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }

    public Servo.Direction getDirection(){
        return servo.getDirection();
    }

    public void servoMove(SafeServoLadderDirections side){
        double position = servo.getPosition();

        if (side.equals(SafeServoLadderDirections.UP)){
            position += 0.01;
        } else {
            position -= 0.01;
        }
        position = Range.clip(position, 0, 1);
        servo.setPosition(position);
    }
}