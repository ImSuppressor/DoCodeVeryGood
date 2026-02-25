package org.firstinspires.ftc.teamcode.tuning;


import static com.qualcomm.robotcore.hardware.DcMotorEx.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.LastPose;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystemsAndMORE.ShootNow;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp
public class LimelightTuneing extends OpMode {
    private PIDController controller;

    public static double kP=0;
    public static double kI=0;
    public static double kD=0;
    private DcMotorEx turret;
    private Limelight3A limelight;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;


    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl.setDirection(Direction.REVERSE);
        fl.setDirection(Direction.REVERSE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        turret = hardwareMap.get(DcMotorEx.class,"turret");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        turret.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        controller = new PIDController(kP, kI, kD);
        kP = .025;
        kI = 0.0001;
        kD = .001;
        controller.setPID(kP, kI, kD);
//        double[] coeffs = controller.getCoefficients();
//        kP = coeffs[0];
//        kI = coeffs[1];
//        kD = coeffs[2];
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(50);



    }
    private void Sticks(double speed) {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        fl.setPower(speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        fr.setPower(-speed * gamepad1.right_stick_x + (-(speed * gamepad1.left_stick_x) - speed * gamepad1.left_stick_y));
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        bl.setPower(speed * gamepad1.right_stick_x + (-speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        br.setPower(-speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        double targetOffsetAngle_Vertical = result.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 90.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 12.0;

        // distance from the target to the floor
        double goalHeightInches = 50.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double DistanceToGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        if (result.isValid() & !(limelight.getLatestResult() == null)) {
//            controller.setPID(kP, kI, kD);
//            kP = .003;
//            kI = 0;
//            kD = .02;
            double turretPosFromTargetInTicks = result.getTx();
            double pidPower = controller.calculate(turretPosFromTargetInTicks, 0);

            turret.setPower(-pidPower);
            telemetry.addData("pos", turret.getCurrentPosition());
            telemetry.addData("target", turretPosFromTargetInTicks);
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("power", pidPower);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("DistanceToGoal",DistanceToGoal);

        }
        else {
            turret.setPower(0);
//            kP = 0;
//            kI = 0;
//            kD = 0;
//            controller.setPID(kP, kI, kD);
        }
            Sticks(.8);

        }
    }
