package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class REVStarterBotTeleOpJava extends LinearOpMode {

    private DcMotor flywheel;
    private DcMotor feeder;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private CRServo intake;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // Setting our velocity targets. These values are in ticks per second!
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotor.class, "motor-flywheel");
        feeder = hardwareMap.get(DcMotor.class, "motor-feeder-A");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left-front-drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left-back-drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right-front-drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right-back-drive");
        intake = hardwareMap.get(CRServo.class, "servo-intake");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Ensures the intake is active and ready
        intake.setPower(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our methods while the OpMode is running
                splitStickArcadeDrive();
                setFlywheelVelocity();
                manualFeederAndIntakeControl();
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();
            }
        }
    }

    /**
     * Controls for the drivetrain. The robot uses a split stick stlye arcade drive.
     * Forward and back is on the left stick. Turning is on the right stick.
     */
    private void splitStickArcadeDrive() {
        double x;
        double y;
        double rx;

        x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    /**
     * Manual control for the Core Hex powered feeder and the agitator servo in the hopper
     */
    private void manualFeederAndIntakeControl() {
        // Manual control for the Core Hex intake
        if (gamepad1.cross) {
            feeder.setPower(0.5);
        } else if (gamepad1.triangle) {
            feeder.setPower(-0.5);
        }
        // Manual control for the hopper's servo
        if (gamepad1.dpad_left) {
            intake.setPower(1);
        } else if (gamepad1.dpad_right) {
            intake.setPower(-1);
        }
    }

    /**
     * This if/else statement contains the controls for the flywheel, both manual and auto.
     * Circle and Square will spin up ONLY the flywheel to the target velocity set.
     * The bumpers will activate the flywheel, Core Hex feeder, and servo to cycle a series of balls.
     */
    private void setFlywheelVelocity() {
        if (gamepad1.options) {
            flywheel.setPower(-0.5);
        } else if (gamepad1.left_bumper) {
            farPowerAuto();
        } else if (gamepad1.right_bumper) {
            bankShotAuto();
        } else if (gamepad1.circle) {
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        } else if (gamepad1.square) {
            ((DcMotorEx) flywheel).setVelocity(maxVelocity);
        } else {
            ((DcMotorEx) flywheel).setVelocity(0);
            feeder.setPower(0);
            // The check below is in place to prevent stuttering with the intake. It checks if the intake is under manual control!
            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                intake.setPower(0);
            }
        }
    }

    /**
     * The bank shot or near velocity is intended for launching balls touching or a few inches from the goal.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The intake will spin until the bumper is released.
     */
    private void bankShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
        intake.setPower(-1);
        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 50) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
    }

    /**
     * The far power velocity is intended for launching balls a few feet from the goal. It may require adjusting the deflector.
     * When running this function, the flywheel will spin up and the Core Hex will wait before balls can be fed.
     * The intake will spin until the bumper is released.
     */
    private void farPowerAuto() {
        ((DcMotorEx) flywheel).setVelocity(farVelocity);
        intake.setPower(-1);
        if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 100) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
    }

}
