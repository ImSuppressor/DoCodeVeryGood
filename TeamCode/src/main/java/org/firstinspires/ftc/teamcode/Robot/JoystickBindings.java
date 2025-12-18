package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class JoystickBindings {
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    public JoystickBindings(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
    }
    public void mainJoystickButtons(Robot robot){
        if(gamepad1.x){

        }
    }
}
