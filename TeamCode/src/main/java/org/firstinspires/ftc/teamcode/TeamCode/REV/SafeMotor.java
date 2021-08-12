package org.firstinspires.ftc.teamcode.TeamCode.REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SafeMotor extends SafeHardware{

    private DcMotor motor;

    private final int TICK_PER_REV_COUNT = 1120;
    private final double GEAR_REDUCTION = 0.5;
    private final int WHEEL_DIAMETER = 4;
    private final int COUNT_PER_INCH = getCountsPerInch(TICK_PER_REV_COUNT, GEAR_REDUCTION, WHEEL_DIAMETER);

    public SafeMotor(HardwareMap hardwareMap, Telemetry telemetry, String name,
                     DcMotor.Direction direction, DcMotor.RunMode run_mode) {
        super(hardwareMap, telemetry, name);

        try {
            motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(direction);
            motor.setMode(run_mode);
        } catch (Exception e) {
            telemetry.addData("Cannot find DC Motor: ", name);
        }

    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public DcMotor.Direction getDirection(){
        return motor.getDirection();
    }

    public void setTargetPosition(int pos) {
        motor.setTargetPosition(pos);
    }

    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void resetMotor() {
        motor.setPower(0);
    }

    public void setMode(DcMotor.RunMode run_mode) {
        motor.setMode(run_mode);
    }

    public DcMotor.RunMode getMode(){
        return motor.getMode();
    }

    public void encoderMove(double inches, double power) {
        int target_position = (int) (inches * COUNT_PER_INCH);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(target_position);
        motor.setPower(power);
        while (motor.isBusy());
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void doubleMotorMove(float button, SafeMotor leftMotor,
                                       SafeMotor rightMotor, int factor){
            double power = button/factor;

            rightMotor.setPower(power);
            leftMotor.setPower(power);
    }

    public int getCountsPerInch(int TICK_PER_REV_COUNT, double GEAR_REDUCTION, int WHEEL_DIAMETER) {
        return (int) ((TICK_PER_REV_COUNT * GEAR_REDUCTION) / (WHEEL_DIAMETER * 3.1415));
    }

    public static void joystickDrive(Gamepad gPad, SafeMotor leftMotor,
                                     SafeMotor rightMotor) {
        if (gPad.right_bumper) {
            leftMotor.setPower(-gPad.left_stick_y);
            rightMotor.setPower(-gPad.right_stick_y);
        } else {
            leftMotor.setPower(-gPad.left_stick_y * 0.75);
            rightMotor.setPower(-gPad.right_stick_y * 0.75);
        }
    }
}