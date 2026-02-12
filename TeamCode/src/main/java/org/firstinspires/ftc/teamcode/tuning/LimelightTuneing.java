package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class,"turret");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);

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

        if (result.isValid() & !(limelight.getLatestResult() == null) & gamepad1.a) {
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

    }
}