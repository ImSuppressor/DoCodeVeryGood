package org.firstinspires.ftc.teamcode.EpsilonOpModes;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.LastPose;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.util.Arrays;

@Autonomous(name="RedDepo", preselectTeleOp = "IOWA_STATE_CHAMPDRIVE")
public class RedDepo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        pattern = "none";
        ColorBay1 = 0;
        ColorBay2 = 0;
        ColorBay3 = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(57,49,Math.toRadians(45)));
        ShooterPID shooterPID = new ShooterPID(hardwareMap);
        ShootNow shootNow = new ShootNow(hardwareMap);
        ShootClose shootClose = new ShootClose(hardwareMap);
        TurretPID turretPID = new TurretPID(hardwareMap);
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx outakeL = hardwareMap.get(DcMotorEx.class, "outakeL");
        DcMotorEx outakeR = hardwareMap.get(DcMotorEx.class, "outakeR");

        outakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outakeR.setDirection(DcMotorEx.Direction.REVERSE);
        outakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo hood = hardwareMap.get(Servo.class, "Hood");
        Servo Boot1 = hardwareMap.get(Servo.class, "Boot1");
        Servo Boot2 = hardwareMap.get(Servo.class, "Boot2");
        Servo Boot3 = hardwareMap.get(Servo.class, "Boot3");
        Boot1.setPosition(.94);
        Boot2.setPosition(.94);
        Boot3.setPosition(.94);
        team=2;
        //TODO:Init
        Limelight3A Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(0);
        Limelight.setPollRateHz(50);
        Limelight.start();
        // Action Scorepre: Y flipped to 49, Heading to 45, Turret to 655
        // Action Scorepre: Y flipped to 49, Heading to 45, Turret to 655
        Action Scorepre = drive.actionBuilder(new Pose2d(57, 49, Math.toRadians(45)))
                .afterTime(.75, new ColorSense(hardwareMap))
                .afterTime(1.1, new SetpositionforMotor(turret, 665)) // Flipped Turret
                .strafeToLinearHeading(new Vector2d(24, 24), Math.toRadians(45))
                .build();

        Action OutakeWheels = drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(new SetvelforMotor(outakeL, 1750))
                .stopAndAdd(new SetvelforMotor(outakeR, 1750))
                .stopAndAdd(new Setpositionforservo(hood, .9))
                .build();

        Action lineUP1 = drive.actionBuilder(new Pose2d(24, 24, 45))
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .strafeToLinearHeading(new Vector2d(30, 8), 0)
                .build();

        Action intakespike = drive.actionBuilder(new Pose2d(30, 6, 0))
                .strafeToLinearHeading(new Vector2d(45, 6), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(10),
                                new AngularVelConstraint(Math.PI / 2)
                        )))
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .build();

        Action score1 = drive.actionBuilder(new Pose2d(67, 6, 0))
                .afterTime(0.25, new SetpositionforMotor(turret, 700)) // Flipped Turret
                .afterTime(.75, new ColorSense(hardwareMap))
                .strafeToLinearHeading(new Vector2d(24, 24), Math.toRadians(45))
                .build();

        Action GoLine2 = drive.actionBuilder(new Pose2d(24, 24, 45))
                .strafeToLinearHeading(new Vector2d(35, -16), Math.toRadians(0))
                .stopAndAdd(new SetpowerforMotor(intake, 0))
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .stopAndAdd(new Setpositionforservo(Boot1, .94))
                .stopAndAdd(new Setpositionforservo(Boot2, .94))
                .stopAndAdd(new Setpositionforservo(Boot3, .94))
                .build();

        Action Grab2 = drive.actionBuilder(new Pose2d(35, -16, 0))
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .strafeToLinearHeading(new Vector2d(55, -16), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(15),
                                new AngularVelConstraint(Math.PI / 2)
                        )))
                .strafeToLinearHeading(new Vector2d(45, -16), 0)
                .stopAndAdd(new SetpowerforMotor(intake, 0))
                .strafeToLinearHeading(new Vector2d(71, 2), 0)
                .afterTime(.75, new ColorSense(hardwareMap))

                .build();

        Action backupDUmp = drive.actionBuilder(new Pose2d(71, -0, 0))
                .strafeToLinearHeading(new Vector2d(35, -5), Math.toRadians(0))
               .build();

        Action Score2 = drive.actionBuilder(new Pose2d(35, -5, 0))
                .afterTime(0.25, new SetpositionforMotor(turret, 715)) // Flipped Turret
                .strafeToLinearHeading(new Vector2d(24, 24), Math.toRadians(45))
                .build();

        Action GoLIne3 = drive.actionBuilder(new Pose2d(24, 24, 45))
                .strafeToLinearHeading(new Vector2d(32, -37), Math.toRadians(0))
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .build();

        Action Grab3 = drive.actionBuilder(new Pose2d(35, -37, 0))
                .stopAndAdd(new SetpowerforMotor(intake, 1))
                .strafeToLinearHeading(new Vector2d(50, -37), Math.toRadians(0),
                        new MinVelConstraint(Arrays.asList(
                                new TranslationalVelConstraint(10),
                                new AngularVelConstraint(Math.PI / 2)
                        )))
                .stopAndAdd(new SetpowerforMotor(intake, 0))

                .build();

        Action backup2 = drive.actionBuilder(new Pose2d(75, -37, 0))
                .strafeToLinearHeading(new Vector2d(40, -38), Math.toRadians(0))
                .build();

        Action Score3 = drive.actionBuilder(new Pose2d(40, -37, 0))
                .afterTime(.75, new ColorSense(hardwareMap))
                .afterTime(0.25, new SetpositionforMotor(turret, 750)) // Flipped Turret
                .strafeToLinearHeading(new Vector2d(24, 26), Math.toRadians(45))
                .stopAndAdd(new SetpowerforMotor(intake, 0))
                .build();

        Action PAARRK = drive.actionBuilder(new Pose2d(24, 24, 45))
                .afterTime(1.5, new SetpowerforMotor(intake, 0))
                .afterTime(0.25, new SetpositionforMotor(turret, 635)) // Flipped Turret
                .strafeToLinearHeading(new Vector2d(24, 30), Math.toRadians(45)) // -180 is effectively 180
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
        Action ServoReset = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new RedDepo.SetpowerforMotor(intake,0))
                .stopAndAdd(new RedDepo.Setpositionforservo(Boot1,.94))
                .stopAndAdd(new RedDepo.Setpositionforservo(Boot2,.94))
                .stopAndAdd(new RedDepo.Setpositionforservo(Boot3,.94))
                .build();
        Action ServoReset2 = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new RedDepo.SetpowerforMotor(intake,0))
                .stopAndAdd(new RedDepo.Setpositionforservo(Boot1,.94))
                .stopAndAdd(new RedDepo.Setpositionforservo(Boot2,.94))
                .stopAndAdd(new RedDepo.Setpositionforservo(Boot3,.94))
                .build();
        Action wait = drive.actionBuilder(new Pose2d(-64, -7, 0))
                        .waitSeconds(2)
                                .build();



        waitForStart();
        //TODO: Run auto
        try {
            //TODO: Run auto
            Actions.runBlocking(new ParallelAction(
                    OutakeWheels, // Runs in background
                    new SequentialAction(
                            Scorepre,
                            shootNow.shoot(),
                            Scorepre,
                            GoLine2,
                            Grab2,
                            //backup,
                            //dump,
                            backupDUmp,
                            Score2,
                            shootNow.shoot(),
                            ServoReset,
                            GoLIne3,
                            Grab3,
                            backup2,
                            Score3,
                            shootNow.shoot(),
                            ServoReset2,
                            lineUP1,
                            intakespike,
                            score1,
                            shootNow.shoot()
                            ,PAARRK
                    )
            ));


        } finally {
            drive.updatePoseEstimate();
            LastPose = drive.localizer.getPose();

        }
    }

    public class SetvelforMotor implements Action {
        DcMotorEx motor;
        double vel;

        public SetvelforMotor(DcMotorEx MotorToCall, double Vel) {
            this.motor = MotorToCall;
            this.vel = Vel;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setVelocity(vel);
            return false;
        }
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




