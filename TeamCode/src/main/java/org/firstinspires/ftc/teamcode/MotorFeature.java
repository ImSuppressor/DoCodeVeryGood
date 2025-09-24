package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public interface MotorFeature {

    public void init(HardwareMap hardwareMap);
    public List<String> driveLoop(Gamepad gamepad1, Gamepad gamepad2);
    public void stop();

    public void goToPosition(Position position);



}
