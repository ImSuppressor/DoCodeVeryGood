package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDF extends OpMode {
    // Creates a PIDFController with gains kP, kI, kD, and kF
    private DcMotorEx outakeL, outakeR;
    private MotorGroup flywheelGroup;
    public PIDFController outakepid;
    public static double P = 0, I = 0, D = 0, F = 0;
    public static double targetVelocity = 1500;

    @Override

    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");

        outakeL.setDirection(DcMotorEx.Direction.FORWARD);
        outakeR.setDirection(DcMotorEx.Direction.REVERSE);

//        flywheelGroup = new MotorGroup(outakeL, outakeR);
//        flywheelGroup.setRunMode(Motor.RunMode.RawPower);

        outakepid = new PIDFController(P, I, D, F);


    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            outakepid.setPIDF(P, I, D, F);


            // Use the velocity from one motor (usually the "master" motor)
            double currentVelocity = outakeL.getVelocity();

            // Calculate power based on that velocity
            double power = outakepid.calculate(currentVelocity, targetVelocity);

            outakeL.setPower(power);
            outakeR.setPower(power);


            telemetry.addData("currentVel", currentVelocity);
            telemetry.addData("tragetVel", targetVelocity);
            telemetry.addData("Power", power);
        } else if (gamepad1.b) {
            outakeL.setPower(.8);
            outakeR.setPower(.8);
            telemetry.addData("currentVel", outakeL.getVelocity());
        } else if (gamepad1.x) {
            outakeL.setVelocity(2300);
            outakeL.setVelocity(2300);
            telemetry.addData("currentVel", outakeL.getVelocity());

        } else if (gamepad1.y){
            outakeL.setPower(0);
            outakeL.setPower(0);
        }
        telemetry.addData("currentVel", outakeL.getVelocity());
        telemetry.update();


    }

}



