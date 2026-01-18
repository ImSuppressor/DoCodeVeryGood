package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

@Config
@TeleOp
public class TurretTune extends OpMode {
    MecanumDrive drive;
    private PIDController TurController;
    private PIDController TurControllerL;
    double P_tur;
    double I_tur;
    double D_tur;
    double P_tur_Lim;
    double I_tur_Lim;
    double D_tur_Lim;
    double normalSpeed;
    double turbo_speed;

    private DcMotorEx turret;
    private Limelight3A limelight;
    private Servo hood;
    private double DisToGoalX;
    private double DisToGoalY;
    private DcMotorEx outakeL;
    private DcMotorEx outakeR;
    public double powervar;
    public double servovar;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private ElapsedTime timer;
    private  Servo Bay1Boot;
    private  Servo Bay2Boot;
    private  Servo Bay3Boot;
    private DcMotorEx intake;
    private GoBildaPinpointDriver pinpoint;


    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-146,82, DistanceUnit.MM);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        Bay1Boot = hardwareMap.get(Servo.class,"Boot1");
        Bay2Boot = hardwareMap.get(Servo.class,"Boot2");
        Bay3Boot = hardwareMap.get(Servo.class,"Boot3");

        timer = new ElapsedTime();

        P_tur = 0.15;
        I_tur = 0;
        D_tur = 0.001;

        P_tur_Lim = .03;
        I_tur_Lim = 0.0015;
        D_tur_Lim = .001;

        normalSpeed = 0.8;
        turbo_speed = 1;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");

        turret = hardwareMap.get(DcMotorEx.class,"turret");

        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        hood = hardwareMap.get(Servo.class,"Hood");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TurController = new PIDController(P_tur, I_tur, D_tur);
        TurControllerL = new PIDController(P_tur_Lim, I_tur_Lim, D_tur_Lim);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(50);

        servovar = .5;
        hood.setPosition(servovar);

        fl = hardwareMap.get(DcMotorEx.class,"fl");
        fr = hardwareMap.get(DcMotorEx.class,"fr");
        bl = hardwareMap.get(DcMotorEx.class,"bl");
        br = hardwareMap.get(DcMotorEx.class,"br");

        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        team = 1;
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

        if (gamepad1.right_trigger > 0) {

            intake.setPower(1);

        } else if (gamepad1.left_trigger > 0) {

            intake.setPower(-1);

        }
        else {

            intake.setPower(0);

        }

        if (gamepad1.dpad_left) {

            Bay1Boot.setPosition(.575);

            Bay2Boot.setPosition(.95);
            Bay3Boot.setPosition(.95);

        }
        else if (gamepad1.dpad_right) {

            Bay2Boot.setPosition(.55);

            Bay1Boot.setPosition(.95);
            Bay3Boot.setPosition(.95);

        }
        else if (gamepad1.dpad_down) {

            Bay3Boot.setPosition(.575);

            Bay1Boot.setPosition(.95);
            Bay2Boot.setPosition(.95);

        }
        else {

            Bay1Boot.setPosition(.95);
            Bay2Boot.setPosition(.95);
            Bay3Boot.setPosition(.95);

        }

        if (gamepad1.right_bumper) {

            Sticks(turbo_speed);

        } else {

            Sticks(normalSpeed);

        }

        pinpoint.update();

        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);
        double currentH = pinpoint.getHeading(AngleUnit.DEGREES);

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {

            timer.reset();

        }


        if (team == 1) {

            double DisToGoalX = 72 - currentX;
            double DisToGoalY = 72 - currentY;
            double DistanceToGoal = Math.hypot(DisToGoalY,DisToGoalX);

            double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;

            double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;

            outakeL.setPower(-powervar);
            outakeR.setPower(powervar);
            hood.setPosition(servovar);

            if (result.isValid() & !(limelight.getLatestResult() == null)) {

                double pidPower = TurControllerL.calculate(result.getTx(), 0);

                double currentPosDeg = turret.getCurrentPosition() * 0.1339285;

                if (currentPosDeg <= 0 && pidPower < 0) {

                    pidPower = 0;

                }

                else if (currentPosDeg >= 177.45 && pidPower > 0) {

                    pidPower = 0;

                }


                turret.setPower(-pidPower);
                telemetry.addData("DistToGoal",DistanceToGoal);

            } else if (!(result.isValid()) & timer.seconds() > .5) {

                double targetAngle = Math.toDegrees(Math.atan2(DisToGoalY, DisToGoalX));

                double currentTurretAngle = turret.getCurrentPosition() * (0.1339285);

                double rawTargetDeg = targetAngle + currentH;

                double clampedTargetDeg = Math.max(0, Math.min(170, rawTargetDeg));

                double pidPower = TurController.calculate(currentTurretAngle, clampedTargetDeg);

                if (currentTurretAngle <= 0 && pidPower < 0) {
                    pidPower = 0;
                }

                if (currentTurretAngle >= 170 && pidPower > 0) {
                    pidPower = 0;
                }

                turret.setPower(pidPower);

                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Current Angle", currentTurretAngle);
                telemetry.addData("PID Output", pidPower);
                telemetry.addData("distance", DistanceToGoal);
                telemetry.addData("powervar", powervar);
                telemetry.addData("servovar", servovar);
                telemetry.addData("DistToGoal",DistanceToGoal);

            } else {

                turret.setPower(0);

            }

        }

        if (team == 2) {

            double DisToGoalX = -(72 - currentX);
            double DisToGoalY = 72 - currentY;
            double DistanceToGoal = Math.hypot(DisToGoalX,DisToGoalY);

            double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;

            double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;

            outakeL.setPower(-powervar);
            outakeR.setPower(powervar);
            hood.setPosition(servovar);

            if (result.isValid() & !(limelight.getLatestResult() == null)) {

                double pidPower = TurControllerL.calculate(result.getTx(), 0);

                double currentPosDeg = turret.getCurrentPosition() * 0.1339285;

                if (currentPosDeg <= 0 && pidPower < 0) {

                    pidPower = 0;

                }

                else if (currentPosDeg >= 177.45 && pidPower > 0) {

                    pidPower = 0;

                }


            } else if (!(result.isValid() & !(limelight.getLatestResult() == null)) & timer.seconds() > .5) {

                double targetAngle = Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

                if (targetAngle < -177.45) {

                    targetAngle = -177.45;

                } else if (targetAngle > 0) {

                    targetAngle = 0;

                }

                double currentTurretAngle = turret.getCurrentPosition() * (0.1339285714285714) + Math.toDegrees(-currentH);

                double pidPower = TurController.calculate(currentTurretAngle, targetAngle);

                turret.setPower(-pidPower);

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

        telemetry.addData("X",currentX);
        telemetry.addData("Y",currentY);
        telemetry.addData("Distx",DisToGoalX);
        telemetry.addData("DistY",DisToGoalY);
        telemetry.addData("Tx",result.getTx());
        telemetry.addData("time",timer.seconds());
        telemetry.update();

    }

}
