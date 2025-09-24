package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class LimeLight implements  MotorFeature{
    Limelight3A limeLight = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        limeLight = hardwareMap.get(Limelight3A.class, "lime");
        limeLight.start();
    }

    @Override
    public List<String> driveLoop(Gamepad gamepad1, Gamepad gamepad2) {

        List<String> telemetryData = new ArrayList();
        telemetryData.add(String.format(Locale.ENGLISH, "",limeLight.getStatus()));
        return telemetryData;
    }

    @Override
    public void stop() {

    }

    @Override
    public void goToPosition(Position position) {

    }
}
