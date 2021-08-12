package org.firstinspires.ftc.teamcode.TeamCode.REV;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SafeCRServo extends SafeHardware{

    private CRServo crServo;

    public SafeCRServo(HardwareMap hardwareMap, Telemetry telemetry, String name,
                       CRServo.Direction direction) {
        super(hardwareMap, telemetry, name);
        try {
            crServo = hardwareMap.get(CRServo.class, name);
            crServo.setDirection(direction);
        } catch (Exception e) {
            telemetry.addData("Cannot find CRServo Sensor: ", name);
        }
    }

    public CRServo.Direction getDirection() {
        return crServo.getDirection();
    }

    public void setDirection(CRServo.Direction direction) {
        crServo.setDirection(direction);
    }

    public double getPower() {
        return crServo.getPower();
    }

    public void setPower(double power) {
        crServo.setPower(power);
    }

    public static void doubleCRServoMove(Gamepad gPad, SafeCRServo leftCRServo,
                                         SafeCRServo rightCRServo){
        double forward_power;
        double backward_power;

        if (gPad.left_trigger > 0) {
            forward_power = gPad.left_trigger;

            leftCRServo.setPower(forward_power);
            rightCRServo.setPower(forward_power);
        }
        if (gPad.right_trigger > 0){
            backward_power = -gPad.right_trigger;

            leftCRServo.setPower(backward_power);
            rightCRServo.setPower(backward_power);
        }
        if (gPad.right_trigger == 0 & gPad.left_trigger == 0){
            leftCRServo.setPower(0);
            rightCRServo.setPower(0);
        }
    }

    public void crServoMove(Gamepad gPad){
        double power = gPad.left_stick_y;

        crServo.setPower(power);
    }
}