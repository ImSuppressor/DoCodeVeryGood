package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DecodeStarterBot extends LinearOpMode {

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    Servo leftfeeder = null;
    Servo rightfeeder = null;
    DcMotor launcher = null;





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




    public void encoderDrive(double dy, double dx) {
        /* 1. Assign the motors
           2. Make a function that takes forwards and backwards inputs
           3. Get all the motors positions and assign to variables
           4. Calculate what the change you need to make for the motors
           5. Tell the motors to move to the new position
           6. End the function
       */

        //dy and dx are negated due to the robot's inversed movement
        double ticks = -dy / CENTIMETERS_PER_TICK;
        double dxticks = 1.1 * (-dx / CENTIMETERS_PER_TICK);




        double frontLeftMotorPos = frontLeftMotor.getCurrentPosition();
        double backLeftMotorPos = backLeftMotor.getCurrentPosition();
        double frontRightMotorPos = frontRightMotor.getCurrentPosition();
        double backRightMotorPos = backRightMotor.getCurrentPosition();

        //dy & dx
        double frontLeftMotorTarget = frontLeftMotorPos + ticks + dxticks;
        double frontRightMotorTarget = frontRightMotorPos + ticks - dxticks;
        double backLeftMotorTarget = backLeftMotorPos + ticks - dxticks;
        double backRightMotorTarget = backRightMotorPos + ticks + dxticks;

        //dy
        frontLeftMotor.setTargetPosition((int) frontLeftMotorTarget);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0.2);

        backLeftMotor.setTargetPosition((int) backLeftMotorTarget);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(0.2);

        frontRightMotor.setTargetPosition((int) frontRightMotorTarget);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(0.2);

        backRightMotor.setTargetPosition((int) backRightMotorTarget);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(0.2);


        //dx
        frontLeftMotor.setTargetPosition((int) frontLeftMotorTarget);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(0.2);

        backLeftMotor.setTargetPosition((int) backLeftMotorTarget);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(0.2);

        frontRightMotor.setTargetPosition((int) frontRightMotorTarget);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(0.2);

        backRightMotor.setTargetPosition((int) backRightMotorTarget);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(0.2);

        //dy & dx
        while ((Math.abs(backLeftMotor.getCurrentPosition() - backLeftMotor.getTargetPosition()) > 5)
                && (Math.abs(backRightMotor.getCurrentPosition() - backRightMotor.getTargetPosition()) > 5)
                && (Math.abs(frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) > 5)
                && Math.abs(frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) > 5) {
            sleep(67);

        }


        telemetry.addData("frontLeftTarget", frontLeftMotorTarget);
        telemetry.addData("frontRightTarget", frontRightMotorTarget);
        telemetry.addData("backLeftTarget", backLeftMotorTarget);
        telemetry.addData("backRightTarget", backRightMotorTarget);

        telemetry.update();

    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Position of the wheel after driving forward

        frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        backRightMotor = hardwareMap.dcMotor.get("right_back_drive");
        leftfeeder = hardwareMap.servo.get("leftfeeder");
        rightfeeder = hardwareMap.servo.get("rightfeeder");
        launcher = hardwareMap.dcMotor.get("launcher");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);






        // Reset the motor encoder so that it reads zero ticks

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

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

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            float rightTrigger = gamepad1.right_trigger;
            float leftTrigger = gamepad1.left_trigger;
            double armPower;



            //Moving Square


            if ((gamepad1.b)){
                leftfeeder.setPosition(0.2);
                rightfeeder.setPosition(-0.2);
            } else if(gamepad1.y){
                leftfeeder.setPosition(-1);
                rightfeeder.setPosition(-1);
            }





            telemetry.addData("frontLeftMotorPos:", frontLeftMotor.getCurrentPosition());
            telemetry.addData("frontRightMotorPos:", frontRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotorPos", backLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotorPos", backRightMotor.getCurrentPosition());

            telemetry.addLine("I have made change");





            telemetry.addLine("pressing dpad Right makes robot go in square");




            telemetry.update();






        }


    }
}


