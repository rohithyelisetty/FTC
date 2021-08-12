package org.firstinspires.ftc.teamcode.TeamCode.REV;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SafeHardware {

    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected String name;

    public SafeHardware(HardwareMap hardwareMap, Telemetry telemetry, String name) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.name = name;
    }

}
