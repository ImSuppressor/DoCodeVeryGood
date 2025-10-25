// File: SimpleAutoLaunch.java
// Put this in your team's opmode package, e.g. org.firstinspires.ftc.teamcode.autonomous
// Requires FTC SDK. Drop into your project and register as an autonomous OpMode.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.text.Format;


@Autonomous(name = "SimpleAutoLaunch", group = "Auton")
public class SimpleAutoLaunch extends LinearOpMode {
    DcMotor lf = null;
    DcMotor rf = null;
    DcMotor lb = null;
    DcMotor rb = null;
    GoBildaPinpointDriver driver = null;
    // Drive motors
    Servo pusher1 = null;
    Servo pusher2 = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    // Shooter motor (use DcMotorEx if you need velocity control)
    private DcMotorEx shooter = null;

    // Servo that "pushes" the ball into the shooter (aka pusher or flicker)


    // Constants you should tune for your robot:
    private static final double WHEEL_DIAMETER_INCHES = 4.0;       // wheel diameter
    private static final double TICKS_PER_REV = 537.6;             // encoder ticks per motor rev (example: NeveRest 20/Orbital 20)
    private static final double GEAR_RATIO = 1.0;                  // output revs per motor rev (1.0 if direct)
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_INCHES);

    // Movement speeds
    private static final double DRIVE_SPEED = 0.6;     // forward/backward speed
    private static final double TURN_SPEED = 0.5;

    // Shooter configuration (tune these)
    private static final double SHOOTER_POWER = 0.9;   // power to spin shooter
    private static final long SHOOTER_SPINUP_MS = 1200; // ms to let shooter spin up before feeding
    private static final long PUSHER_FORWARD_MS = 300;  // servo forward time to push ball
    private static final long PUSHER_BACK_MS = 300;     // servo back time to reset
    private static final double PUSHER_FORWARD_POS = 0.2; // servo position for pushing
    private static final double PUSHER_RETRACT_POS = 0.8; // servo position for retracted

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Map hardware - change names to exactly match your robot config
        DcMotor lf  = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor lb  = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rb = hardwareMap.get(DcMotor.class, "right_back_drive");

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        shooter    = hardwareMap.get(DcMotorEx.class, "launcher");
        Servo pusher1 = (Servo) hardwareMap.get(Servo.class, "leftfeeder");
        Servo pusher2     = (Servo) hardwareMap.get(Servo.class, "rightfeeder");

        // Set directions - change if your motors are reversed
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop and configure encoders
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Initialize servo to retracted position
        pusher1.setPosition(0.8);
        pusher2.setPosition(0.8);

        telemetry.addData("Status", "Init complete");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Example autonomous sequence:
        // 1) Drive forward X inches
        // 2) Spin shooter and wait for spin-up
        // 3) Use servo to push ball into shooter (one or more times)
        // 4) Drive backward Y inches

        // 1) Drive forward 36 inches (tune distance)
        encoderDrive(DRIVE_SPEED, 5, 5.0); // speed, inches, timeoutSec

        // small pause
        sleep(200);

        // 2) Spin shooter
        shooter.setPower(SHOOTER_POWER);
        sleep(SHOOTER_SPINUP_MS);

        // 3) Push ball (feed)
        pushBallOnce();

        // Optional: push multiple times if you have multiple balls
        // sleep(600); // optional delay between pushes
        // pushBallOnce();

        // Stop shooter
        shooter.setPower(0);

        // 4) Drive backward 12 inches
        encoderDrive(DRIVE_SPEED, -5.0, 4.0);

        // End of autonomous
        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    /**
     * Drives a set distance (inches) using encoders on left and right motors.
     * Positive inches -> forward. Negative -> backward.
     * timeoutSec is maximum seconds to allow.
     *
     */
    private void encoderStrafe(double speed, double inches, double timeoutSec){
        int moveCounts = (int)Math.round((inches * COUNTS_PER_INCH));

        int StrafeCurrent = driver.getEncoderX();

        int newSt = StrafeCurrent + moveCounts;

        lf.setTargetPosition(newSt);
        rf.setTargetPosition(newSt);
        rb.setTargetPosition(newSt);
        lb.setTargetPosition(newSt);


    }
    private void encoderDrive(double speed, double inches, double timeoutSec) {
        int newLeftTarget;
        int newRightTarget;

        // Calculate target ticks
        int moveCounts = (int)Math.round(inches * COUNTS_PER_INCH);

        // Current positions
//        int leftfrontCurrent = lf.getCurrentPosition();
//        int leftbackCurrent = lb.getCurrentPosition();
//        int rightfrontCurrent = rf.getCurrentPosition();
//        int rightbackCurrent = rb.getCurrentPosition();

        int FowardCurrent = driver.getEncoderY();




        int newLt = FowardCurrent + moveCounts;


        // Set target and run to position
        lf.setTargetPosition(newLt);
        rf.setTargetPosition(newLt);
        rb.setTargetPosition(newLt);
        lb.setTargetPosition(newLt);


        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set power
        lb.setPower(Math.abs(speed));
        rb.setPower(Math.abs(speed));
        lf.setPower(Math.abs(speed));
        rf.setPower(Math.abs(speed));

        // timeout handling
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (leftDrive.isBusy() || rightDrive.isBusy())) {
            // Optionally add telemetry

            // allow the loop to be interrupted
            idle();
        }

        // Stop motors
        lb.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        rf.setPower(0);

        // Switch back to run using encoder
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Pushes the ball into the shooter once using the pusher servo.
     */
    private void pushBallOnce() {
        // Move pusher forward
        pusher1.setPosition(PUSHER_FORWARD_POS);
        sleep(PUSHER_FORWARD_MS);
        pusher2.setPosition(0);

        pusher2.setPosition(PUSHER_FORWARD_POS);
        sleep(PUSHER_FORWARD_MS);
        pusher2.setPosition(0);



        // Retract pusher
        pusher1.setPosition(PUSHER_RETRACT_POS);
        sleep(PUSHER_BACK_MS);
        pusher1.setPosition(0);

        pusher2.setPosition(PUSHER_RETRACT_POS);
        sleep(PUSHER_BACK_MS);
        pusher2.setPosition(0);
    }
}