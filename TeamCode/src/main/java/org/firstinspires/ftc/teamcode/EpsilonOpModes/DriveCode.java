package org.firstinspires.ftc.teamcode.EpsilonOpModes;


import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

@Config
@TeleOp
public class DriveCode extends OpMode {
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
    private ElapsedTime timer2;
    private  Servo Bay1Boot;
    private  Servo Bay2Boot;
    private  Servo Bay3Boot;
    private DcMotorEx intake;
    private GoBildaPinpointDriver pinpoint;
    private Pose2D Pose2ding;
    private NormalizedColorSensor bay11;
    private NormalizedColorSensor bay12;
    private NormalizedColorSensor bay21;
    private NormalizedColorSensor bay22;
    private NormalizedColorSensor bay31;
    private NormalizedColorSensor bay32;
    private DistanceSensor dist11, dist12, dist21, dist22, dist31, dist32;
    private boolean initialized = false;
    private boolean shooting1 = false;
    private boolean shooting2 = false;
    private boolean shooting3 = false;
    public double shoot = .5;

    public double ready = 0.97;

    public double cycle = .5;
    public double cycleDown = .12;
    public double ball1, ball2, ball3;
    public boolean Boot1 = false;
    public boolean Boot2 = false;
    public boolean Boot3 = false;
    public boolean SequenceShoot = false;


    @Override
    public void init() {
        bay11 = hardwareMap.get(NormalizedColorSensor.class, "Bay1.1");
        bay12 = hardwareMap.get(NormalizedColorSensor.class, "Bay1.2");
        dist11 = hardwareMap.get(DistanceSensor.class, "Bay1.1");
        dist12 = hardwareMap.get(DistanceSensor.class, "Bay1.2");
        bay21 = hardwareMap.get(NormalizedColorSensor.class, "Bay2.1");
        bay22 = hardwareMap.get(NormalizedColorSensor.class, "Bay2.2");
        dist21 = hardwareMap.get(DistanceSensor.class, "Bay2.1");
        dist22 = hardwareMap.get(DistanceSensor.class, "Bay2.2");
        bay31 = hardwareMap.get(NormalizedColorSensor.class, "Bay3.1");
        bay32 = hardwareMap.get(NormalizedColorSensor.class, "Bay3.2");
        dist31 = hardwareMap.get(DistanceSensor.class, "Bay3.1");
        dist32 = hardwareMap.get(DistanceSensor.class, "Bay3.2");


        Pose2ding = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,172.5);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(82,-146, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(Pose2ding);
        team=1;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);


        Bay1Boot = hardwareMap.get(Servo.class,"Boot1");
        Bay2Boot = hardwareMap.get(Servo.class,"Boot2");
        Bay3Boot = hardwareMap.get(Servo.class,"Boot3");
        Bay1Boot.setPosition(.94);
        Bay2Boot.setPosition(.94);Bay3Boot.setPosition(.94);

        timer = new ElapsedTime();

        P_tur = 0.1;//ticks per degree
        I_tur = 0;
        D_tur = 0.001;//power given

        P_tur_Lim = .03;
        I_tur_Lim = 0.0015;
        D_tur_Lim = .001;

        normalSpeed = 0.8;
        turbo_speed = 1;

//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");
        outakeL.setDirection(DcMotorSimple.Direction.REVERSE);

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
        pattern="PPG";
        if (pattern.equals("PPG")) {
            ball1 = 1;
            ball2 = 1;
            ball3 = 2;

        } else if (pattern.equals("PGP")) {
            ball1 = 1;
            ball2 = 2;
            ball3 = 1;

        } else if (pattern.equals("GPP")) {
            ball1 = 2;
            ball2 = 1;
            ball3 = 1;

        }

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
//        if ( bay11 == 1&&gamepad1.dpad_left) {
//            Bay1Boot.setPosition(.535);
//            bay11 = 0;
//            Bay1Boot = true;
//            timer.reset();
//        }


        if (gamepad1.right_trigger > 0) {

            intake.setPower(1);

        } else if (gamepad1.left_trigger > 0) {

            intake.setPower(-1);

        }
        else {

            intake.setPower(0);

        }
        if ( gamepad1. square){
            outakeL.setPower(.55);
            outakeR.setPower(.55);
        }
        if (gamepad1. triangle) {
            outakeR.setPower(.63);
            outakeL.setPower(.63);
            hood.setPosition(.64);
        }
        if( gamepad1.circle){
            outakeL.setPower(.725);
            outakeR.setPower(.725
            );
            hood.setPosition(.9);
        }
        if ( gamepad1.a){
            outakeL.setPower(0);
            outakeR.setPower(0);
            hood.setPosition(.54);
        }
        if (gamepad1.right_bumper){
            intake.setPower(.8);
        }
//        if (gamepad1.dpad_left) {
//
//            Bay1Boot.setPosition(.575);
//
//            Bay2Boot.setPosition(.95);
//            Bay3Boot.setPosition(.95);
//
//        }
//        else if (gamepad1.dpad_right) {
//
//            Bay2Boot.setPosition(.55);
//
//            Bay1Boot.setPosition(.95);
//            Bay3Boot.setPosition(.95);
//
//        }
//        else if (gamepad1.dpad_down) {
//
//            Bay3Boot.setPosition(.575);
//
//            Bay1Boot.setPosition(.95);
//            Bay2Boot.setPosition(.95);
//
//        }
//        else {
//
//            Bay1Boot.setPosition(.95);
//            Bay2Boot.setPosition(.95);
//            Bay3Boot.setPosition(.95);
//
//        }

        if (gamepad1.right_bumper) {

            Sticks(turbo_speed);

        } else {

            Sticks(normalSpeed);

        }

        Pose2D currentPos = pinpoint.getPosition();

        double currentX = currentPos.getX(DistanceUnit.INCH);
        double currentY = currentPos.getY(DistanceUnit.INCH);
        double currentH = currentPos.getHeading(AngleUnit.DEGREES);

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {

            timer.reset();

        }


        if (team == 1) {

            double DisToGoalX = 72 - currentX;
            double DisToGoalY = 72 + currentY;

            double DistanceToGoal = Math.hypot(DisToGoalX,DisToGoalY);

//            double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;
//
//            double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;
//
//            outakeL.setPower(-powervar);
//            outakeR.setPower(powervar);
//            hood.setPosition(servovar);
//            telemetry.addData("Distx",DisToGoalX);
//            telemetry.addData("DistY",DisToGoalY);
//            telemetry.addData("Tx",result.getTx());

            if (result.isValid() & !(limelight.getLatestResult() == null)) {

                double pidPower = TurControllerL.calculate(result.getTx(), 0);

                double currentPosDeg = (turret.getCurrentPosition() * 0.1339285) +7.5;

                if (currentPosDeg <= 7.5 && pidPower < 0) {

                    pidPower = 0;

                }

                else if (currentPosDeg >= 172.5 && pidPower > 0) {

                    pidPower = 0;

                }


                turret.setPower(-pidPower);
                telemetry.addData("DistToGoal",DistanceToGoal);

            } else if (!(result.isValid()) & timer.seconds() > .5) {

                double targetAngle = Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

                double currentTurretAngle = (turret.getCurrentPosition() * (0.1339285)) + 7.5;

                double rawTargetDeg = targetAngle + currentH;

                double clampedTargetDeg = Math.max(7.5, Math.min(172.5, rawTargetDeg));

                double pidPower = TurController.calculate(currentTurretAngle, clampedTargetDeg);

                if (currentTurretAngle <= 7.5 && pidPower < 0) {
                    pidPower = 0;
                }

                if (currentTurretAngle >= 172.5 && pidPower > 0) {
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

            double DisToGoalX = 72 - currentX;
            double DisToGoalY = 72 + currentY;

            double DistanceToGoal = Math.hypot(DisToGoalX,DisToGoalY);

            double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;

            double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;

            outakeL.setPower(-powervar);
            outakeR.setPower(powervar);
            hood.setPosition(servovar);
            telemetry.addData("Distx",DisToGoalX);
            telemetry.addData("DistY",DisToGoalY);
            telemetry.addData("Tx",result.getTx());

            if (result.isValid() & !(limelight.getLatestResult() == null)) {

                double pidPower = TurControllerL.calculate(result.getTx(), 0);

                double currentPosDeg = (turret.getCurrentPosition() * 0.1339285) -7.5;

                if (currentPosDeg >= -7.5 && pidPower < 0) {

                    pidPower = 0;

                }

                else if (currentPosDeg <= -172.5 && pidPower > 0) {

                    pidPower = 0;

                }


                turret.setPower(-pidPower);
                telemetry.addData("DistToGoal",DistanceToGoal);

            } else if (!(result.isValid()) & timer.seconds() > .5) {

                double targetAngle = -Math.toDegrees(Math.atan2(DisToGoalX, DisToGoalY));

                double currentTurretAngle = (turret.getCurrentPosition() * (0.1339285)) - 7.5;

                double rawTargetDeg = targetAngle + currentH;

                double clampedTargetDeg = Math.max(-172.5, Math.min(-7.5, rawTargetDeg));

                double pidPower = TurController.calculate(currentTurretAngle, clampedTargetDeg);

                if (currentTurretAngle <= 7.5 && pidPower < 0) {
                    pidPower = 0;
                }

                if (currentTurretAngle >= 172.5 && pidPower > 0) {
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
        double distance2 = Math.min(dist21.getDistance(DistanceUnit.CM), dist22.getDistance(DistanceUnit.CM));
        double distance1 = Math.min(dist11.getDistance(DistanceUnit.CM), dist12.getDistance(DistanceUnit.CM));
        double distance3 = Math.min(dist31.getDistance(DistanceUnit.CM), dist32.getDistance(DistanceUnit.CM));

        if (distance1 < 3) {
            NormalizedRGBA colors11 = bay11.getNormalizedColors();
            NormalizedRGBA colors12 = bay12.getNormalizedColors();
            double avgBlue1 = (colors11.blue + colors12.blue) / 2.0;
            double avgGreen1 = (colors11.green + colors12.green) / 2.0;
            if (avgBlue1 > avgGreen1) {
                ColorBay1 = 1;
            } else if (avgGreen1 > avgBlue1) {
                ColorBay1 = 2;
            } else {
                ColorBay1 = 0;
            }
        }
        if (distance2 < 3) {
            NormalizedRGBA colors21 = bay21.getNormalizedColors();
            NormalizedRGBA colors22 = bay22.getNormalizedColors();
            double avgBlue2 = (colors21.blue + colors22.blue) / 2.0;
            double avgGreen2 = (colors21.green + colors22.green) / 2.0;
            if (avgBlue2 > avgGreen2) {
                ColorBay2 = 1;
            } else if (avgGreen2 > avgBlue2) {
                ColorBay2 = 2;//bay 2 is stipud
            } else {
                ColorBay2 = 0;
            }
        }
        if (distance3 < 10) {
            NormalizedRGBA colors31 = bay31.getNormalizedColors();
            NormalizedRGBA colors32 = bay32.getNormalizedColors();
            double avgBlue3 = (colors31.blue + colors32.blue) / 2.0;
            double avgGreen3 = (colors31.green + colors32.green) / 2.0;
            if (avgBlue3 > avgGreen3) {
                ColorBay3 = 1;
            } else if (avgGreen3 > avgBlue3) {
                ColorBay3 = 2;
            } else {
                ColorBay3 = 0;
            }
        }

        //ShootPurple
        if (!shooting1&&!shooting2&&gamepad1.dpad_left && !Boot1 && !Boot2 && !Boot3) {
            if (ColorBay1 == 1) {
                Bay1Boot.setPosition(shoot);
                ColorBay1 = 0;
                timer.reset();
                Boot1 = true;

            } else if (ColorBay2 == 1) {
                Bay2Boot.setPosition(shoot);
                ColorBay2 = 0;
                timer.reset();
                Boot2 = true;

            } else if (ColorBay3 == 1) {
                Bay3Boot.setPosition(shoot);
                ColorBay3 = 0;
                timer.reset();
                Boot3 = true;

            }
        }
        //ShootGreen
        if (gamepad1.dpad_right && !Boot1 && !Boot2 && !Boot3) {
            if (ColorBay1 == 2) {
                Bay1Boot.setPosition(shoot);
                ColorBay1 = 0;
                timer.reset();
                Boot1 = true;

            } else if (ColorBay2 == 2) {
                Bay2Boot.setPosition(shoot);
                ColorBay2 = 0;
                timer.reset();
                Boot2 = true;

            } else if (ColorBay3 == 2) {
                Bay3Boot.setPosition(shoot);
                ColorBay3 = 0;
                timer.reset();
                Boot3 = true;

            }
        }
        //Reset Boot Arms
        if (timer.seconds() < cycle && timer.seconds() > (cycle-cycleDown) && (Boot1 || Boot2 || Boot3)) {
            Bay1Boot.setPosition(ready);
            Bay2Boot.setPosition(ready);
            Bay3Boot.setPosition(ready);
        } else if (timer.seconds() > cycle && (Boot1 || Boot2 || Boot3)) {
            Boot1 = false;
            Boot2 = false;
            Boot3 = false;
        }
        //SequencedShot
        if (gamepad2.dpad_down || SequenceShoot == true) {
            if (!initialized) {
              //  timer2.reset();
                SequenceShoot = true;
            }

            if (timer2.seconds() < (cycle) && !shooting1) {

                if (ColorBay1 == ball1) {

                    Bay1Boot.setPosition(shoot);

                    ColorBay1 = 0;
                    shooting1 = true;

                } else if (ColorBay2 == ball1) {

                    Bay2Boot.setPosition(shoot);

                    ColorBay2 = 0;
                    shooting1 = true;

                } else if (ColorBay3 == ball1) {

                    Bay3Boot.setPosition(shoot);

                    ColorBay3 = 0;
                    shooting1 = true;

                } else {
                    if (!(ColorBay1 == 0)) {
                        Bay1Boot.setPosition(shoot);

                        ColorBay1 = 0;
                        shooting1 = true;
                    } else if (!(ColorBay2 == 0)) {
                        Bay2Boot.setPosition(shoot);

                        ColorBay2 = 0;
                        shooting1 = true;

                    } else if (!(ColorBay3 == 0)) {
                        Bay3Boot.setPosition(shoot);

                        ColorBay3 = 0;
                        shooting1 = true;

                    }
                }
            } else if (timer2.seconds() > (cycle-cycleDown) && timer2.seconds() < cycle) {
                Bay1Boot.setPosition(ready);
                Bay2Boot.setPosition(ready);
                Bay3Boot.setPosition(ready);

            } else if (timer2.seconds() < 2 * cycle && timer2.seconds() > cycle && !shooting2) {

                if (ColorBay1 == ball2) {

                    Bay1Boot.setPosition(shoot);

                    ColorBay1 = 0;
                    shooting2 = true;

                } else if (ColorBay2 == ball2) {

                    Bay2Boot.setPosition(shoot);

                    ColorBay2 = 0;
                    shooting2 = true;

                } else if (ColorBay3 == ball2) {

                    Bay3Boot.setPosition(shoot);

                    ColorBay3 = 0;
                    shooting2 = true;

                } else {
                    if (!(ColorBay1 == 0)) {
                        Bay1Boot.setPosition(shoot);

                        ColorBay1 = 0;
                        shooting2 = true;
                    } else if (!(ColorBay2 == 0)) {
                        Bay2Boot.setPosition(shoot);

                        ColorBay2 = 0;
                        shooting2 = true;

                    } else if (!(ColorBay3 == 0)) {
                        Bay3Boot.setPosition(shoot);

                        ColorBay3 = 0;
                        shooting2 = true;

                    }
                }

            }  else if (timer2.seconds() > ((2*cycle)-cycleDown) && timer2.seconds() < 2*cycle) {
                Bay1Boot.setPosition(ready);
                Bay2Boot.setPosition(ready);
                Bay3Boot.setPosition(ready);

            } else if (timer2.seconds() < 3 * cycle && timer2.seconds() > 2 * cycle && !shooting3) {

                if (ColorBay1 == ball3) {

                    Bay1Boot.setPosition(shoot);

                    ColorBay1 = 0;
                    shooting3 = true;

                } else if (ColorBay2 == ball3) {

                    Bay2Boot.setPosition(shoot);

                    ColorBay2 = 0;
                    shooting3 = true;

                } else if (ColorBay3 == ball3) {

                    Bay3Boot.setPosition(shoot);

                    ColorBay3 = 0;
                    shooting3 = true;

                } else {
                    if (!(ColorBay1 == 0)) {
                        Bay1Boot.setPosition(shoot);

                        ColorBay1 = 0;
                        shooting3 = true;
                    } else if (!(ColorBay2 == 0)) {
                        Bay2Boot.setPosition(shoot);

                        ColorBay2 = 0;
                        shooting3 = true;

                    } else if (!(ColorBay3 == 0)) {
                        Bay3Boot.setPosition(shoot);

                        ColorBay3 = 0;
                        shooting3 = true;
                        timer.reset();

                    }
                }
            }
            if (timer2.seconds() > 3*cycle && initialized) {
                shooting1 = false;
                shooting2 = false;
                shooting3 = false;
                initialized = false;
                SequenceShoot = false;
            }
        }

        telemetry.addData("Bay 3", ColorBay3);
        telemetry.addData("Bay 2", ColorBay2);
        telemetry.addData("Bay 1", ColorBay1);
        telemetry.addData("time", timer.seconds());
        telemetry.addData("currentX",currentX);
        telemetry.addData("currentY",currentY);
        telemetry.addData("currentH",currentH);
        telemetry.addData("currentPos",currentPos);
        telemetry.update();

    }

}
