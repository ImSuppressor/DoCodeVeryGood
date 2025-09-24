package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Launcher implements MotorFeature {
    DcMotor launchMotor = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        launchMotor = hardwareMap.get(DcMotor.class, "launch");
    }


    @Override
    public List<String> driveLoop(Gamepad gamepad1, Gamepad gamepad2) {

        if (gamepad1.a) {
            launchMotor.setPower(100);
        }

        List<String> telemetryData = new ArrayList();
        telemetryData.add(String.format(Locale.ENGLISH, "Intake x2 %4.2f", launchMotor.getPower()));
        return telemetryData;
    }

    @Override
    public void stop() {

    }

    @Override
    public void goToPosition(Position position) {

    }
}
