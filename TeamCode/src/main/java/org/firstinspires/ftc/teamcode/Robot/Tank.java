package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Tank {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    /*private static final double fixedAngleRatio = 4/5;
    private static final double tolerance = 0.05;*/
    private static final double powerPerVelocity = 0.5;
    public final double halfOfBodyWidth;
    public Tank(DcMotor rightMotor, DcMotor leftMotor, double bodyWidth){
        this.leftMotor=leftMotor;
        this.rightMotor=rightMotor;
        this.halfOfBodyWidth = bodyWidth/2;
    }
    /*
    public void goToPosition(double degrees, double power){
        double reasonablePower = 0.6*power;
        if(!(MathUtilBlitz.isNear(0,degrees,tolerance)||MathUtilBlitz.isNear(180, reasonablePower, tolerance)&& degrees>0)){
            rightMotor.setPower(reasonablePower*fixedAngleRatio);
            leftMotor.setPower(reasonablePower);
        } else if(!(MathUtilBlitz.isNear(0, degrees, tolerance) || MathUtilBlitz.isNear(180, degrees, tolerance))){
            rightMotor.setPower(reasonablePower);
            leftMotor.setPower(reasonablePower*fixedAngleRatio);
        }
        else{
            leftMotor.setPower(reasonablePower);
            rightMotor.setPower(reasonablePower);
        }
    }
     */
    //differential implementation, see graph at: https://www.desmos.com/calculator/4amzxia3wr (ctrl click to open)
    public void moveWithAngleVelocity(double angle, double bodyVelocity){
        double leftVelocity = bodyVelocity + angle*halfOfBodyWidth;
        double rightVelocity = bodyVelocity - angle*halfOfBodyWidth;
        rightMotor.setPower(rightVelocity*powerPerVelocity);
        leftMotor.setPower(leftVelocity*powerPerVelocity);
    }
    public void moveWithStickXY(double stickX, double stickY){
        moveWithAngleVelocity(Math.atan2(stickY,stickX),Math.sqrt(stickX*stickX+stickY*stickY));
        //note that the robot doesnt go backwards it moves forwards but the angle is 180degrees
    }
    public void stop(){
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
