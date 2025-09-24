package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class MotorDrive implements MotorFeature {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DeviceNames names;
    public void init(HardwareMap hardwareMap){

        leftFrontDrive = hardwareMap.get(DcMotor.class, names.LF_MOTOR.toString());
        leftBackDrive = hardwareMap.get(DcMotor.class, names.LB_MOTOR.toString());
        rightFrontDrive = hardwareMap.get(DcMotor.class, names.RF_MOTOR.toString());
        rightBackDrive = hardwareMap.get(DcMotor.class, names.RB_MOTOR.toString());


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    public List<String> driveLoop(Gamepad gamepad1, Gamepad gamepad2){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad2.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad2.left_stick_x;
        double yaw     =  gamepad2.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        List <String> telemetryData = new ArrayList<>();
        telemetryData.add(String.format(Locale.ENGLISH, "Front left/Right %4.2f, %4.2f", leftFrontPower, rightFrontPower));
        telemetryData.add(String.format(Locale.ENGLISH, "Back  left/Right %4.2f, %4.2f", leftBackPower, rightBackPower));

        return telemetryData;
    }

    @Override
    public void stop() {

    }

    @Override
    public void goToPosition(Position position) {

    }
}