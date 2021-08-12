package org.firstinspires.ftc.teamcode.TeamCode.REV;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class SafeIMU extends SafeHardware{

    private BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private Orientation angles;

    public SafeIMU(@NonNull HardwareMap hardwareMap, Telemetry telemetry, String name){
        super(hardwareMap, telemetry, name);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        try {
            imu = hardwareMap.get(BNO055IMU.class, name);
            angles = new Orientation();
        } catch (Exception e) {
            telemetry.addData("Cannot find Gyro Sensor: ", name);
        }
        imu.initialize(parameters);

    }

    public int getZValue(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }

    public int getYValue() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }

    public int getXValue(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return (int) angles.firstAngle;
    }

    public void imuStraight(SafeMotor leftMotor, SafeMotor rightMotor,
                            int degree, double power, double inches){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int curr_degree = (int) angles.firstAngle;
        int distance = (int) (inches * rightMotor.getCountsPerInch
                (1120, 0.5, 4));
        int error;

        while (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(distance) &&
                Math.abs(rightMotor.getCurrentPosition()) < Math.abs(distance)){
            error = degree - curr_degree;

            if (Math.abs(error) > 3) {
                leftMotor.resetMotor();
                rightMotor.resetMotor();
                imuTurn(leftMotor, rightMotor, error, 2);
            }

            power = Range.clip(power, -1, 1);
            leftMotor.setPower(power);
            rightMotor.setPower(power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            curr_degree = (int) angles.firstAngle;
        }

        leftMotor.resetMotor();
        rightMotor.resetMotor();
    }

    public void imuTurn(SafeMotor leftMotor, SafeMotor rightMotor, int degree, int error){
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double curr_position = angles.firstAngle;

        while (curr_position != Range.clip(curr_position,degree - error, degree + error)){
            double power = (degree - curr_position)/150;

            if (power < 0.1 && power > 0){
                power = 0.1;
            }

            if (power > -0.1 && power < 0){
                power = -0.1;
            }

            rightMotor.setPower(power);
            leftMotor.setPower(-power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            curr_position = (int) angles.firstAngle;
        }

        leftMotor.resetMotor();
        rightMotor.resetMotor();
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
