package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.Scanning;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

public class TurretPID {
    private final HardwareMap hwMap;

    public final double P_tur = 0;
    public final double I_tur = 0;
    public final double D_tur = 0;
    public final double P_tur_Lim = .05;
    public final double I_tur_Lim = 0.00015;
    public final double D_tur_Lim = .001;

    private final DcMotorEx turret;
    private final Limelight3A limelight;
    private MecanumDrive drive;
    private final DcMotorEx outakeL;
    private final DcMotorEx outakeR;
    private final Servo hood;

        public TurretPID(HardwareMap hardwareMap) {
            this.hwMap = hardwareMap;
            // Initialize hardware in the main class constructor
            this.turret = hwMap.get(DcMotorEx.class, "turret");
            this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.limelight = hwMap.get(Limelight3A.class, "limelight");
            this.outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
            this.outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");
            this.hood = hardwareMap.get(Servo.class,"Hood");
        }
//1235 is ticks buddy boy
        public class HomeTurret implements Action {
    private final PIDController TurController;
    private final PIDController TurControllerL;
    private double DisToGoalX = 0;
    private double DisToGoalY = 0;
    public HomeTurret() {
        TurController = new PIDController(P_tur, I_tur, D_tur);
        TurControllerL = new PIDController(P_tur_Lim, I_tur_Lim, D_tur_Lim);
        drive = new MecanumDrive(hwMap, new Pose2d(-64, -7, 0));

        limelight.start();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(50);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        drive.updatePoseEstimate();
        Pose2d currentPose = drive.localizer.getPose();
        double currentX = currentPose.position.x;
        double currentY = currentPose.position.y;
        double currentH = currentPose.heading.toDouble();
        LLResult result = limelight.getLatestResult();


        if (team == 1) {
            double DisToGoalX = 72 + currentX;
            double DisToGoalY = 72 - currentY;
            double DistanceToGoal = Math.hypot(DisToGoalX,DisToGoalY);

            double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;

            double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;

            outakeL.setPower(-powervar);
            outakeR.setPower(powervar);
            hood.setPosition(servovar);

            if (result.isValid() & !(limelight.getLatestResult() == null)) {
                double pidPower = TurControllerL.calculate(result.getTx(), 0);

                turret.setPower(pidPower);

            } else if (!(result.isValid() & !(limelight.getLatestResult() == null))) {

                double targetAngle = Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

                if (targetAngle > 177.45) {
                    targetAngle = 177.45;

                } else if (targetAngle < 0) {
                    targetAngle = 0;

                }

                double currentTurretAngle = turret.getCurrentPosition() * (0.1339285) + Math.toDegrees(-currentH);

                double pidPower = TurController.calculate(currentTurretAngle, targetAngle);

                turret.setPower(pidPower);

                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Current Angle", currentTurretAngle);
                telemetry.addData("PID Output", pidPower);
                telemetry.addData("distance", DistanceToGoal);
                telemetry.addData("powervar", powervar);
                telemetry.addData("servovar", servovar);
            } else {
                turret.setPower(0);
            }
        }
        if (team == 2) {
            double DisToGoalX = 72 - currentX;
            double DisToGoalY = 72 - currentY;
            double DistanceToGoal = Math.hypot(DisToGoalX,DisToGoalY);

            double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;

            double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;

            outakeL.setPower(-powervar);
            outakeR.setPower(powervar);
            hood.setPosition(servovar);

            if (result.isValid() & !(limelight.getLatestResult() == null)) {



            } else if (!(result.isValid() & !(limelight.getLatestResult() == null))) {

                double targetAngle = Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

                if (targetAngle > 177.45) {
                    targetAngle = 177.45;

                } else if (targetAngle < 0) {
                    targetAngle = 0;

                }

                double currentTurretAngle = turret.getCurrentPosition() * (0.1339285714285714) + Math.toDegrees(-currentH);

                double pidPower = TurController.calculate(currentTurretAngle, targetAngle);

                turret.setPower(pidPower);

                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Current Angle", currentTurretAngle);
                telemetry.addData("PID Output", pidPower);
                telemetry.addData("distance", DistanceToGoal);
                telemetry.addData("powervar", powervar);
                telemetry.addData("servovar", servovar);
            } else {
                turret.setPower(0);
            }
        }
        return true; //Error < .15 & Error > -.15;//Error Tolerance That Determines Shutoff


    }
}

    public Action homeTurret() {

        return new HomeTurret();
    }
}
