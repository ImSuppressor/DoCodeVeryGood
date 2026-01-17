package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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

import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

public class TurretPID {
    private final HardwareMap hwMap;

    public final double P_tur = 0;
    public final double I_tur = 0;
    public final double D_tur = 0;
    public final double P_tur_Lim = .05;
    public final double I_tur_Lim = 0.00015;
    public final double D_tur_Lim = .001;

    private DcMotorEx turret;
    private Limelight3A limelight;
    private MecanumDrive drive;

        public TurretPID(HardwareMap hardwareMap) {
            this.hwMap = hardwareMap;
            // Initialize hardware in the main class constructor
            this.turret = hwMap.get(DcMotorEx.class, "turret");
            this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.limelight = hwMap.get(Limelight3A.class, "limelight");
        }
//1235 is ticks buddy boy
        public class HomeTurret implements Action {
    private PIDController TurController;
    private double ErrorInDegrees = 0;
    private double DisToGoalX = 0;
    private double DisToGoalY = 0;
    public HomeTurret() {
        TurController = new PIDController(P_tur, I_tur, D_tur);
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

        if (!Scanning) {
            if (Math.abs(ErrorInDegrees) < 15) {
                if (result != null && result.isValid()) {
                    TurController.setPID(P_tur_Lim, I_tur_Lim, D_tur_Lim);
                    double turretPosFromTargetInDegrees = result.getTx();
                    double PIDturret = TurController.calculate(turretPosFromTargetInDegrees, 0);
                    turret.setPower(-PIDturret);
                }
            } else {
                TurController.setPID(P_tur, I_tur, D_tur);
                double PIDturretP = TurController.calculate(ErrorInDegrees, 0);
                turret.setPower(PIDturretP);
            }
            if (team == 1) { // red
                if (currentX < 0) DisToGoalX = -currentX + 72;
                else if (currentX > 0) DisToGoalX = currentX;

                if (currentY < 0) DisToGoalY = -currentY + 72;
                else if (currentY > 0) DisToGoalY = currentY;
            }
            if (team == 2) { // blue
                if (currentX > 0) DisToGoalX = -currentX - 72;
                else if (currentX < 0) DisToGoalX = currentX;

                if (currentY > 0) DisToGoalY = -currentY - 72;
                else if (currentY < 0) DisToGoalY = currentY;
            }
        }
        double pinpoint_error = Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

        packet.put("pinpoint_error", ErrorInDegrees);
        packet.put("turretPos", turret.getCurrentPosition());
        return true; //Error < .15 & Error > -.15;//Error Tolerance That Determines Shutoff


    }
}

    public Action homeTurret() {

        return new HomeTurret();
    }
}
