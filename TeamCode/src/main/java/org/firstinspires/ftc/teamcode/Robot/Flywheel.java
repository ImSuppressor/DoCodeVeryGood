package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Flywheel {
    private final DcMotor flywheelMotor;

    private static final double spinWhileNotShooting = 0.2;

    private static final double spinWhileShooting = 0.8;

    public Flywheel(DcMotor flywheelMotor){
        this.flywheelMotor=flywheelMotor;
    }

    public void normalSpin(){
        flywheelMotor.setPower(spinWhileNotShooting);
    }

    public void whenShooting(){
        flywheelMotor.setPower(spinWhileShooting);
    }

    public void stop(){
        flywheelMotor.setPower(0);
    }
}
