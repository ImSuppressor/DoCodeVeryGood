package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class testCodeForIntakeShootaTransfer extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
//    DcMotor frontLeftMotor = null;
//    DcMotor backLeftMotor = null;
//    DcMotor frontRightMotor = null;
//    DcMotor backRightMotor = null;
    DcMotor transfer = null;
    DcMotor leftintake = null;
    DcMotor rightintake = null;
    DcMotor launcher = null;
//    Servo deciderofdoom = null;
    ColorSensor csensor = null;

    public String detectColor() {
        // Get raw color values
        int red = csensor.red();
        int green = csensor.green();
        int blue = csensor.blue();

        // Normalize for lighting
        double total = red + green + blue;
        double r = red / total;
        double g = green / total;
        double b = blue / total;

        // Detect GREEN (high green value, low red/blue)
        if (g > 0.45 && r < 0.35 && b < 0.35) {
            return "Green";
        }
        // Detect PURPLE (high red & blue mix, low green)
        else if (r > 0.35 && b > 0.35 && g < 0.3) {
            return "Purple";
        }
        else {
            return "Unknown";
        }
    }





    double wheelCircumfrence = 326.56;
    //Ticks Per Revolution
    double TPR = 537.7;
    //Centimeters Per Tick
    double CENTIMETERS_PER_TICK = 0.0607;


    // Gradually ramps motor power towards a target
    public void rampMotorPower(DcMotor motor, double targetPower, double rampRate) {
        double currentPower = motor.getPower();

        if (currentPower < targetPower) {
            currentPower += rampRate;
            if (currentPower > targetPower) {
                currentPower = targetPower;
            }
        } else if (currentPower > targetPower) {
            currentPower -= rampRate;
            if (currentPower < targetPower) {
                currentPower = targetPower;
            }
        }

        motor.setPower(currentPower);
    }







    @Override
    public void runOpMode() throws InterruptedException {

        VisionPortal webcam;

        //Hardwaremap

        //Control Hub
//        frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
//        backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
//        frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
//        backRightMotor = hardwareMap.dcMotor.get("right_back_drive");

        //Expansion Hub
        transfer = hardwareMap.dcMotor.get(("transfer"));
         leftintake = hardwareMap.dcMotor.get("leftintake");
         rightintake = hardwareMap.dcMotor.get("rightintake");
//        intake = hardwareMap.dcMotor.get(("intake"));
        launcher = hardwareMap.dcMotor.get("launcher");
        csensor = hardwareMap.colorSensor.get("colorsensor");
//        deciderofdoom = hardwareMap.servo.get(("deciderservo"));


        // Set motor settings
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);







        // Sets IMU

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        /*IMPORTANT!!!!!!!!!!!

         */


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX + rx) / denominator)*0.5;
            double backLeftPower = ((rotY - rotX + rx) / denominator)*0.5;
            double frontRightPower = ((rotY - rotX - rx) / denominator)*0.5;
            double backRightPower = ((rotY + rotX - rx) / denominator)*0.5;

//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

            //Code for Actions
            /*
            IMPORTANT!!!!!
            using the left trigger warmups launcher
            pressing a makes the intake spin for 2.5 seconds
            and makes the color sensor sense the artifacts color, which the servo will move
            and load the artifact into the transfer
            pressing b triggers the transfer to run for 2.5 secs

             */

            // Warmup Launcher Code
            float leftTrigger = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;




             if(gamepad1.x){
                 //ts works
                leftintake.setPower(1);
                rightintake.setPower(1);
                transfer.setPower(1);
                 telemetry.update();
            } else{
                leftintake.setPower(0);
                rightintake.setPower(0);
                transfer.setPower(0);
            }




//             if(leftTrigger > 0){
//                 launcher.setPower(1);
//             } else if(leftTrigger == 0){
//                 launcher.setPower(0);
//             }
//
//            if(rightTrigger > 0){
//                launcher.setPower(-1);
//            } else if(rightTrigger == 0){
//                launcher.setPower(0);
//            }

//            if(gamepad1.left_bumper){
//                launcher.setPower(0.5);
//            } else
//
           if(gamepad1.right_bumper){
                //ts correct
                launcher.setPower(-0.5);
            } else {
                launcher.setPower(0);
            }
















//            telemetry.addData("frontLeftMotorPos:", frontLeftMotor.getCurrentPosition());
//            telemetry.addData("frontRightMotorPos:", frontRightMotor.getCurrentPosition());
//            telemetry.addData("backLeftMotorPos", backLeftMotor.getCurrentPosition());
//            telemetry.addData("backRightMotorPos", backRightMotor.getCurrentPosition());

            telemetry.addLine("I have made change");

            telemetry.addData("Color Sensor Values", detectColor());




            telemetry.addLine("pressing dpad Right makes robot go in square");




            telemetry.update();






        }


    }
}


