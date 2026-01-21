package org.firstinspires.ftc.teamcode.EpsilonOpModes;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystemsAndMORE.ShootNow;
import org.firstinspires.ftc.teamcode.SubSystemsAndMORE.ShooterPID;
import org.firstinspires.ftc.teamcode.SubSystemsAndMORE.TurretPID;

@Autonomous(name="BlueDepot", preselectTeleOp = "Drive26")
public class BlueDepot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        pattern = "none";
        ColorBay1 = 0;
        ColorBay2 = 0;
        ColorBay3 = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(57, -49, Math.toRadians(-45)));
        ShooterPID shooterPID = new ShooterPID(hardwareMap);
        ShootNow shootNow = new ShootNow(hardwareMap);
        TurretPID turretPID = new TurretPID(hardwareMap);
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        DcMotorEx outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");

        outakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo hood = hardwareMap.get(Servo.class, "Hood");
        Servo Boot1 = hardwareMap.get(Servo.class, "Boot1");
        Servo Boot2 = hardwareMap.get(Servo.class, "Boot2");
        Servo Boot3 = hardwareMap.get(Servo.class, "Boot3");
        Boot1.setPosition(.95);
        Boot2.setPosition(.95);
        Boot3.setPosition(.95);

        //TODO:Init
        Limelight3A Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(0);
        Limelight.setPollRateHz(50);
        Limelight.start();


        //TODO:Init Everything cracka
        Action Scorepre = drive.actionBuilder(new Pose2d(57, -49, Math.toRadians(-45)))//move to park
                .stopAndAdd(new SetpowerforMotor(outakeL,.55))
                .stopAndAdd(new SetpowerforMotor(outakeR,.55))
                .stopAndAdd(new Setpositionforservo(hood,.45))
                .afterTime(.75,new ColorSense(hardwareMap))
                .afterTime(1.1,new SetpositionforMotor(turret,-617))
                .strafeToLinearHeading(new Vector2d(24, -24), Math.toRadians(-45))
                //.waitSeconds(1.5)
                .build();
        Action lineUP1 = drive.actionBuilder(new Pose2d(24, -24, -45))
                .stopAndAdd(new SetpowerforMotor(intake,1))
                .strafeToLinearHeading(new Vector2d(30,-10),0)
                .build();

                //.build();
        Action intakespike = drive.actionBuilder(new Pose2d(30, -10, 0))
                .strafeToLinearHeading(new Vector2d(65,-10),0)
                .stopAndAdd(new SetpowerforMotor(intake,1))
                .build();

        Action reset = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .waitSeconds(4)
                .build();
        Action score1 = drive.actionBuilder(new Pose2d(65, -10, 0))
                .afterTime(.75,new ColorSense(hardwareMap))
                .strafeToLinearHeading(new Vector2d(24,-24),Math.toRadians(-45))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .build();
        Action GoLine2 = drive.actionBuilder(new Pose2d(24, -12, -45))
                .strafeToLinearHeading(new Vector2d(20,20),Math.toRadians(0))
                .build();
        Action Grab2 = drive.actionBuilder(new Pose2d(20,20,0))
                .strafeToLinearHeading(new Vector2d(65,20),0)
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .build();
        Action Score2 = drive.actionBuilder(new Pose2d(65,20,0))
                .afterTime(.75,new ColorSense(hardwareMap))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .strafeToLinearHeading(new Vector2d(24,-24),Math.toRadians(-45))
                        .build();
        Action GoLIne3 = drive.actionBuilder(new Pose2d(24,-24,-45))
                .strafeToLinearHeading(new Vector2d(24,40),Math.toRadians(0))
                .stopAndAdd(new SetpowerforMotor(intake,1))
                .build();
        Action Grab3 =drive.actionBuilder(new Pose2d(24,40,0))
                .strafeToLinearHeading(new Vector2d(65,40),0)
                .build();
        Action Score3 = drive.actionBuilder(new Pose2d(65,40,0))
                .afterTime(.75,new ColorSense(hardwareMap))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .strafeToLinearHeading(new Vector2d(24,-24),Math.toRadians(-45))
                .build();
        Action IN = drive.actionBuilder(new Pose2d(24, -24, -45))
                .stopAndAdd(new SetpowerforMotor(intake,1))

                .build();




        Action Waiting = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .waitSeconds(1)
                .build();
        Action serest = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .stopAndAdd(new Setpositionforservo(Boot1,.95))
                .stopAndAdd(new Setpositionforservo(Boot2,.95))
                .stopAndAdd(new Setpositionforservo(Boot3,.95))
                .build();
        Action one = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new Setpositionforservo(Boot1,.575))
                .afterTime(.2,new ColorSense(hardwareMap))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .build();
        Action two = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new Setpositionforservo(Boot2,.575))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .afterTime(.2,new ColorSense(hardwareMap))
                .build();
        Action three = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new Setpositionforservo(Boot3,.575))
                .stopAndAdd(new SetpowerforMotor(intake,0))
                .afterTime(.2,new ColorSense(hardwareMap))
                .build();
        Action color = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .afterTime(.75,new ColorSense(hardwareMap))

                .build();
        Action PAARRK = drive.actionBuilder(new Pose2d(24,-24,-45))
                .strafeToLinearHeading(new Vector2d(0,0),Math.toRadians(0))
                        .build();



        while (!isStopRequested() && !opModeIsActive()) {
            LLResult result = Limelight.getLatestResult();
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                telemetry.addData("AprilTag", fiducial.getFiducialId());
                telemetry.addData("Bay 3", ColorBay3);
                telemetry.addData("Bay 2", ColorBay2);
                telemetry.addData("Bay 1", ColorBay1);
                telemetry.update();
            }
        }

        waitForStart();
        //TODO: Run auto
        Actions.runBlocking(new SequentialAction(
              Scorepre
        ));
        Actions.runBlocking(new SequentialAction(
                Waiting,
                shootNow.shoot()
        ));
//        Actions.runBlocking(new SequentialAction(
//        ));
        Actions.runBlocking(new SequentialAction(
               serest
                ));
        Actions.runBlocking(new SequentialAction(

                lineUP1
        ));
        Actions.runBlocking(new SequentialAction(
                serest
        ));
        Actions.runBlocking(new SequentialAction(
                IN,
                intakespike
        ));
        Actions.runBlocking(new SequentialAction(
               score1
        ));
        Actions.runBlocking(new SequentialAction(
                shootNow.shoot()



        ));
        Actions.runBlocking(new SequentialAction(
               GoLine2,
                serest
        ));
        Actions.runBlocking(new SequentialAction(
                IN,
                Grab2
        ));
        Actions.runBlocking(new SequentialAction(
                Score2
        ));
        Actions.runBlocking(new SequentialAction(
                shootNow.shoot(),
                serest

        ));
        Actions.runBlocking(new SequentialAction(
                IN,
                GoLIne3
        ));
        Actions.runBlocking(new SequentialAction(
                Grab3
        ));
        Actions.runBlocking(new SequentialAction(
                Score3

        ));
        Actions.runBlocking(new SequentialAction(
                shootNow.shoot(),
                serest

        ));
        Actions.runBlocking(new SequentialAction(
                PAARRK,
                Waiting

        ));




//        Actions.runBlocking(new SequentialAction(//place spec 1
//                Detect,
//                one,
//                two,
//                three,
//                reset
//        ));
//        Actions.runBlocking(shooterPID.spinUp());
    }


    public class SetpositionforMotor implements Action {
        DcMotorEx motor;
        int position;
        ElapsedTime timer;
        boolean init = false;

        public SetpositionforMotor(DcMotorEx s, int p) {
            this.motor = s;
            this.position = p;
            this.timer = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!init) {
                timer.reset();
                init = true;
            }
            motor.setTargetPosition(position);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return timer.seconds() < .5;
        }
    }
    public class SetpowerforMotor implements Action {
        DcMotorEx motor;
        double power;

        public SetpowerforMotor(DcMotorEx MotorToCall, double Power) {
            this.motor = MotorToCall;
            this.power = Power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setPower(power);
            return false;
        }
    }
    public class Setpositionforservo implements Action {
        Servo servo;
        double position;

        public Setpositionforservo(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }


    public class ColorSense implements Action {
        private final NormalizedColorSensor bay11, bay12, bay21, bay22, bay31, bay32;
        private final DistanceSensor dist11, dist12, dist21, dist22, dist31, dist32;
        private final Limelight3A Limelight;
        private boolean initialized = false;
        private final ElapsedTime timer;


        public ColorSense(HardwareMap hwMap) {
            this.bay11 = hwMap.get(NormalizedColorSensor.class, "Bay1.1");
            this.bay12 = hwMap.get(NormalizedColorSensor.class, "Bay1.2");
            this.dist11 = hwMap.get(DistanceSensor.class, "Bay1.1");
            this.dist12 = hwMap.get(DistanceSensor.class, "Bay1.2");
            this.bay21 = hwMap.get(NormalizedColorSensor.class, "Bay2.1");
            this.bay22 = hwMap.get(NormalizedColorSensor.class, "Bay2.2");
            this.dist21 = hwMap.get(DistanceSensor.class, "Bay2.1");
            this.dist22 = hwMap.get(DistanceSensor.class, "Bay2.2");
            this.bay31 = hwMap.get(NormalizedColorSensor.class, "Bay3.1");
            this.bay32 = hwMap.get(NormalizedColorSensor.class, "Bay3.2");
            this.dist31 = hwMap.get(DistanceSensor.class, "Bay3.1");
            this.dist32 = hwMap.get(DistanceSensor.class, "Bay3.2");
            this.Limelight = hwMap.get(Limelight3A.class, "limelight");
            this.timer = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                ColorBay1 = 0;
                ColorBay2 = 0;
                ColorBay3 = 0;
                timer.reset();
                initialized = true; // Prevents this block from running again
            }
            LLResult result = Limelight.getLatestResult();
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (pattern.equals("none")) {
                    if (fiducial.getFiducialId() == 21) {
                        pattern = "GPP";
                    } else if (fiducial.getFiducialId() == 22) {
                        pattern = "PGP";
                    } else if (fiducial.getFiducialId() == 23) {
                        pattern = "PPG";
                    }
                }
                telemetry.addData("pattern", pattern);
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
            telemetry.addData("Bay 3", ColorBay3);
            telemetry.addData("Bay 2", ColorBay2);
            telemetry.addData("Bay 1", ColorBay1);
            telemetry.addData("time", timer.seconds());
            telemetry.update();


            return timer.seconds() < .4;
        }


    }
}




