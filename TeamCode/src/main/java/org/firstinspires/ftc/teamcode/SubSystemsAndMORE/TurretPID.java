package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretPID {
    private PIDController OutController;
    private PIDController TurController;

    public double P_tur = 0;
    public double I_tur = 0;
    public double D_tur = 0;
    public double P_tur_Lim = 0;
    public double I_tur_Lim = 0;
    public double D_tur_Lim = 0;

    private DcMotorEx turret;
    private Limelight3A limelight;

        public TurretPID(HardwareMap hardwareMap) {
            turret = hardwareMap.get(DcMotorEx.class, "turret");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            limelight = hardwareMap.get(Limelight3A.class,"limelight");
            limelight.start();
            limelight.pipelineSwitch(0);
            limelight.setPollRateHz(50);
        }

        public class HomeTurret implements Action {
            private double PIDturret;
            private double ErrorInDegrees;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                LLResult result = limelight.getLatestResult();
                if (ErrorInDegrees < 15) {
                    ///LimelightHoming
                    if (result.isValid() & !(limelight.getLatestResult() == null)) {
                        TurController.setPID(P_tur_Lim, I_tur_Lim, D_tur_Lim);

                        double turretPosFromTargetInDegrees = result.getTx();

                        double PIDturret = TurController.calculate(turretPosFromTargetInDegrees, 0);
                    }
                } else if (ErrorInDegrees > 15) {
                    ///RoadRunnerHoming

                }

                double Pos = turret.getCurrentPosition();
                packet.put("turretPos", Pos);
                double Error = ErrorInDegrees;
                return Error < .15 & Error > -.15;//Error Tolerance That Determines Shutoff
            }
        }

        public Action homeTurret() {
            return new HomeTurret();
        }
    }
