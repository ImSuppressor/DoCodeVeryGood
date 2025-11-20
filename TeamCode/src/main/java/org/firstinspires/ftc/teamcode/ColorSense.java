package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSense extends OpMode {
    TestColorJava sensor  = new TestColorJava();
    TestColorJava.detectedColor detectedColor;
    @Override
    public void init(){
        sensor.init(hardwareMap);
    }
    @Override
    public void loop(){
        detectedColor = sensor.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);

    }
}
