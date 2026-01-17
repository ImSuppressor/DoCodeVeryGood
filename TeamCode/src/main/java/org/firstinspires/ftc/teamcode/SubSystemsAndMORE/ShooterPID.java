package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

public class ShooterPID {
    private PIDController OutController;
    public double P_out = 0;
    public double I_out = 0;
    public double D_out = 0;
    private DcMotorEx outakeL;
    private DcMotorEx outakeR;
    public ShooterPID(HardwareMap hardwareMap) {
        outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");
    }

    public class SpinUp implements Action {
        private boolean initialized = false;
        private double PIDoutake;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                outakeL.setVelocity(PIDoutake);
                outakeR.setVelocity(PIDoutake);
            }

            double vel = outakeL.getVelocity();
            packet.put("shooterVelocity", vel);
            return vel < 10_000.0;
        }
    }

    public Action spinUp() {

        return new SpinUp();
    }
}