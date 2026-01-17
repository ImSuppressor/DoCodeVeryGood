package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

@Config
@TeleOp
public class TurretTune extends OpMode {
    MecanumDrive drive;
    private PIDController controller;

    public static double kP=0.15;
    public static double kI=0;
    public static double kD=0.0001;
    private DcMotorEx turret;
    private Limelight3A limelight;
    private Servo hood;
    private double ErrorInDegrees = 0;
    private double DisToGoalX = 0;
    private double DisToGoalY = 0;
    private DcMotorEx outakeL;
    private DcMotorEx outakeR;
    public double powervar;
    public double servovar;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");
        turret = hardwareMap.get(DcMotorEx.class,"turret");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        hood = hardwareMap.get(Servo.class,"Hood");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDController(kP, kI, kD);
        kP = .15;
        kI = 0.0000;
        kD = .0001;
        controller.setPID(kP, kI, kD);
//        double[] coeffs = controller.getCoefficients();
//        kP = coeffs[0];
//        kI = coeffs[1];
//        kD = coeffs[2];
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        limelight.start();
//        limelight.pipelineSwitch(0);
//        limelight.setPollRateHz(50);
servovar = .5;
        hood.setPosition(servovar);
    }

    @Override
    public void loop() {

        controller.setPID(kP, kI, kD);
        LLResult result = limelight.getLatestResult();
        drive.updatePoseEstimate();
        Pose2d currentPose = drive.localizer.getPose();
        double currentX = currentPose.position.x;
        double currentY = currentPose.position.y;
        double currentH = currentPose.heading.toDouble();
        double DisToGoalX = 72 + currentX;
        double DisToGoalY = 72 - currentY;
        double DistanceToGoal = Math.hypot(DisToGoalX,DisToGoalY);
        outakeL.setPower(powervar);
        outakeR.setPower(-powervar);
        hood.setPosition(servovar);

        if (gamepad1.dpad_up) {
            powervar = powervar +.025;

        }
        if (gamepad1.dpad_down) {
            powervar = powervar -.025;

        }
        if (gamepad1.dpad_right) {
            servovar = servovar +.025;
        }
        if (gamepad1.dpad_left) {
            servovar = servovar -.025;

        }

        if (!gamepad1.touchpad) {
            // 3. Calculate target angle in degrees
            double targetAngle = Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

            // 4. Calculate current turret angle (you need a ticks-to-degrees conversion here)
            double currentTurretAngle = turret.getCurrentPosition() * (0.1472)+Math.toDegrees(-currentH);

            // 5. Run PID: calculate(current, target)
            double pidPower = controller.calculate(currentTurretAngle, targetAngle);

            // Clamp power to avoid burning out motor while tuning
            turret.setPower(pidPower);

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentTurretAngle);
            telemetry.addData("PID Output", pidPower);
            telemetry.addData("distance",DistanceToGoal);
            telemetry.addData("powervar",powervar);
            telemetry.addData("servovar",servovar);
        } else {
            turret.setPower(0);
        }
        telemetry.update();
    }
}
