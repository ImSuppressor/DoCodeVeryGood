package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestColorJava {
    NormalizedColorSensor colorSensor;

    public enum detectedColor{
        RED,
        BLUE,
        YELLOW,
        GREEN,
        PURPLE,
        UNKNOWN

    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class,"colorsensor");
        colorSensor.setGain(4);

    }



    public detectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();// returns 4 values

        float normRed,normGreen,normBlue;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;
        normRed = colors.red/colors.alpha;


        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        telemetry.addLine("Mugil Neethi Official Watermark:  XXX?YLiYguM?XXX");
        telemetry.update();

        if(normRed > 0.35 && normGreen < 0.3 && normBlue < 0.3){

            return detectedColor.RED;
        }
        else if(normRed > 0.5 && normGreen > 0.9 && normBlue < 0.6){

            return detectedColor.YELLOW;

        } else if (normRed < 0.2 && normGreen < 0.5 && normBlue > 0.5 ) {

            return detectedColor.BLUE;

        } else if (normGreen > normRed * 1.3 && normGreen > normBlue * 1.3 && normGreen > 0.2) {

            return detectedColor.GREEN;

        } else if (normRed > 0.2 && normBlue > 0.2 && normGreen < 0.15) {

            return detectedColor.PURPLE;

        } else {
            return detectedColor.UNKNOWN;
        }

    }
}
