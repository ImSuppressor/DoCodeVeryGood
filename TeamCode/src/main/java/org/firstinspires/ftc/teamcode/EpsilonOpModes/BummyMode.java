package org.firstinspires.ftc.teamcode.EpsilonOpModes;

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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

@Config
@TeleOp
public class BummyMode extends OpMode {
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
    private Pose2D Pose2ding;


    @Override
    public void init() {
        Pose2ding = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,180);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(82,-146, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(Pose2ding);

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

//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

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
        limelight.pipelineSwitch(2);
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
        double cycleDown = .75;

        team = 1;
    }
    /// 165 degrees is turret
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
        pinpoint.update();

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

            Bay2Boot.setPosition(.97);
            Bay3Boot.setPosition(.97);

        }
        else if (gamepad1.dpad_right) {

            Bay2Boot.setPosition(.55);

            Bay1Boot.setPosition(.97);
            Bay3Boot.setPosition(.97);

        }
        else if (gamepad1.dpad_down) {

            Bay3Boot.setPosition(.575);

            Bay1Boot.setPosition(.97);
            Bay2Boot.setPosition(.97);

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
        if(gamepad1.a){
            hood.setPosition(.75);
        }
        if (gamepad1.b){
            hood.setPosition(1);
        }
        if(gamepad1.x){
            hood.setPosition(.45);
        }
        if (gamepad1.dpad_up){
            outakeL.setPower(-.535);
            outakeR.setPower(.535);
        }
        if (gamepad1.right_bumper){
            outakeL.setPower(0);
            outakeR.setPower(0);
        }
//
//        double currentX = currentPos.getX(DistanceUnit.INCH);
//        double currentY = currentPos.getY(DistanceUnit.INCH);
//        double currentH = currentPos.getHeading(AngleUnit.DEGREES);
//
//        LLResult result = limelight.getLatestResult();
//
//        if (result.isValid()) {
//
//            timer.reset();
//
//        }


    }

}

