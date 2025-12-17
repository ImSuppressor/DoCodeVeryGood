package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;

public class Robot {
    private static final Rail rail = new Rail(hardwareMap.crservo.get("railLeft"), hardwareMap.crservo.get("railRight"));

    public Rail getRail() {
        return rail;
    }
}
