package org.firstinspires.ftc.teamcode.TeamCode.REV;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SafeColorDistance extends SafeHardware{


    private ColorSensor color;
    private DistanceSensor distance;

    public SafeColorDistance(@NonNull HardwareMap hardwareMap, Telemetry telemetry, String name) {
        super(hardwareMap, telemetry, name);

        try {
            color = hardwareMap.get(ColorSensor.class, name);
            distance = hardwareMap.get(DistanceSensor.class, name);
        } catch (Exception e) {
            telemetry.addData("Cannot find Color Distance Sensor: ", name);
        }
    }

    public void colorStop(SafeMotor leftMotor, SafeMotor rightMotor, int red, int green,
                          int blue, int error, double power){
        int[] target_color = {red, green, blue};

        int[] curr_color = currColor();

        while (!colorWithinRange(target_color, curr_color, error)){
            leftMotor.setPower(power);
            rightMotor.setPower(power);

            curr_color = currColor();
        }

        leftMotor.resetMotor();
        rightMotor.resetMotor();
    }

    private boolean colorWithinRange(int[] target_color, int[] curr_color, int error){
        for (int k = 0; k < target_color.length; k++){
            if (Math.abs(curr_color[k] - target_color[k]) > error){
                return false;
            }
        }
        return true;
    }

    private int scaleColor(int color) {
        return color * 255;
    }

    private int[] scaleColor(int[] colors) {
        for (int k = 0; k < colors.length; k++){
            colors[k] = (colors[k] * 255);
        }

        return colors;
    }

    private int[] currColor(){
        int[] curr_color = new int[3];
        curr_color[0] = color.red();
        curr_color[1] = color.green();
        curr_color[2] = color.blue();

        return scaleColor(curr_color);
    }

    public void distanceStop(SafeMotor leftMotor, SafeMotor rightMotor,
                             int distance, double power, double error){
        double curr_distance = this.distance.getDistance(DistanceUnit.CM);
        while (curr_distance != Range.clip(curr_distance, distance - error, distance + error)){
            if (curr_distance > distance){
                rightMotor.setPower(power);
                leftMotor.setPower(power);
            }
            if (distance > curr_distance){
                rightMotor.setPower(-power);
                leftMotor.setPower(-power);
            }
            curr_distance = this.distance.getDistance(DistanceUnit.CM);
        }
        rightMotor.resetMotor();
        leftMotor.resetMotor();
    }

    public double getDistance(DistanceUnit unit){
        return distance.getDistance(unit);
    }
}