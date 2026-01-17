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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        ColorBay1 = 1;
        ColorBay2 = 2;
        ColorBay3 = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-64, -7, 0));
        ShooterPID shooterPID = new ShooterPID(hardwareMap);
        ShootNow shootNow = new ShootNow(hardwareMap);
        TurretPID turretPID = new TurretPID(hardwareMap);

        //TODO:Init
        Limelight3A Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(0);
        Limelight.setPollRateHz(50);
        Limelight.start();

        ElapsedTime time2;


        //TODO:Init Everything cracka
        Action Detect = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
                .stopAndAdd(new ColorSense(hardwareMap))
                .waitSeconds(2)
//                .stopAndAdd(new ColorSense("off"))
//                .waitSeconds(1)
//                .stopAndAdd(new ShootBall("Shoot1"))
//                .waitSeconds(1)
//                .stopAndAdd(new ColorSense("Shoot2"))
//                .waitSeconds(1)
//                .stopAndAdd(new ShootBall("Shoot3"))
//                .waitSeconds(25)
                .build();

        Action Score = drive.actionBuilder((new Pose2d(-55,54,0)))// scoreing pos
                .strafeToLinearHeading(new Vector2d(-63, 32), 0)
                .build();
        Action lineUpOne = drive.actionBuilder(new Pose2d(-55,54,0))
                .strafeToLinearHeading(new Vector2d(0, 40), 0)
                .build();
        Action lineONE = drive.actionBuilder(new Pose2d(-63,32,0))
                .strafeToLinearHeading(new Vector2d(-48,0),0)
                .build();
        Action shootLineONe = drive.actionBuilder(new Pose2d(-48,0,0))
                .strafeToLinearHeading(new Vector2d(-62,32),0)
                .build();
        Action LIneTWO = drive.actionBuilder(new Pose2d(-62,32,0))
                .strafeToLinearHeading(new Vector2d(-48,12.5),0)
                .build();
        Action shooLIneTWo = drive.actionBuilder(new Pose2d(-48,12.5,0))
                .strafeToLinearHeading(new Vector2d(-62,32),0)
                .build();
        Action LIentree = drive.actionBuilder(new Pose2d(-62,32,0))// 48 36 0
                .strafeToLinearHeading(new Vector2d(-48,36),0)



                .build();
        Action ShootLINETHREe = drive.actionBuilder(new Pose2d(-48,-36,0))
                .strafeToLinearHeading(new Vector2d(-63,32),0)
                .build();
        Action PAAAAAAAAAAAAAARRRRRRRRRRRRRRRRRRK =drive.actionBuilder(new Pose2d(-63,32,0))
                .strafeToLinearHeading(new Vector2d(-40,0),0)
                .build();


//        Action one = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
//                .stopAndAdd(new ColorSense("off"))
//                .waitSeconds(1)
//                .stopAndAdd(new ShootBall("Shoot1"))
//                .waitSeconds(1)
//
//                .build();
//        Action two = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
//                .stopAndAdd(new ShootBall("Shoot2"))
//                .waitSeconds(1)
//
//                .build();
//        Action three = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
//                .stopAndAdd(new ShootBall("Shoot3"))
//                .waitSeconds(1)
//
//                .build();
//        Action reset = drive.actionBuilder(new Pose2d(-64, -7, 0))
//                .stopAndAdd(new ShootBall("Done"))
//                .waitSeconds(4)
//                .build();
        Action Waiting = drive.actionBuilder(new Pose2d(-64,-7,0))
                .waitSeconds(5)
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
//                shooterPID.spinUp(),
                new SequentialAction(
//                        Detect,
//                        shootNow.shoot(),
                        turretPID.homeTurret(),
                        Waiting
//                        one,
//                        two,
//                        three,
//                        reset
                )
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




//    public class ShootBall implements Action {
//        Servo Bay1Boot = hardwareMap.get(Servo.class, "Boot1");
//        Servo Bay2Boot = hardwareMap.get(Servo.class, "Boot2");
//        Servo Bay3Boot = hardwareMap.get(Servo.class, "Boot3");
//        double shoot;
//        double ready;
//        double cycle;
//        String ShootState;
//
//        public ShootBall(String ShootState) {
//            this.ShootState = ShootState;
//            this.shoot = .6;
//            this.ready = .95;
//            this.cycle = 1;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//            if (pattern.equals("PPG")) {
//                if (ShootState.equals("Shoot1")) {
//                    telemetry.addLine("ball1");
//                    if (ColorBay1.equals("Purple1")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Purple1")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Purple1")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//                if (ShootState.equals("Shoot2")) {
//                    telemetry.addLine("ball2");
//
//                    if (ColorBay1.equals("Purple2")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Purple2")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Purple2")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//                if (ShootState.equals("Shoot3")) {
//                    telemetry.addLine("ball3");
//                    if (ColorBay1.equals("Green1")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Green1")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Green1")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//            }
//            if (pattern.equals("PGP")) {
//                if (ShootState.equals("Shoot1")) {
//
//                    if (ColorBay1.equals("Purple1")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Purple1")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Purple1")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//                if (ShootState.equals("Shoot2")) {
//
//                    if (ColorBay1.equals("Green1")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Green1")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Green1")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//                if (ShootState.equals("Shoot3")) {
//
//                    if (ColorBay1.equals("Purple2")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Purple2")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Purple2")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//            }
//
//
//            if (pattern.equals("GPP")) {
//                if (ShootState.equals("Shoot1")) {
//                    if (ColorBay1.equals("Green1")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Green1")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Green1")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//                if (ShootState.equals("Shoot2")) {
//
//                    if (ColorBay1.equals("Purple1")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Purple1")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Purple1")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//                if (ShootState.equals("Shoot3")) {
//                    if (ColorBay1.equals("Purple2")) {
//
//                        Bay2Boot.setPosition(ready);
//                        Bay3Boot.setPosition(ready);
//
//                        Bay1Boot.setPosition(shoot);
//                    } else if (ColorBay2.equals("Purple2")) {
//                        Bay1Boot.setPosition(ready);
//
//                        Bay3Boot.setPosition(ready);
//
//                        Bay2Boot.setPosition(shoot);
//                    } else if (ColorBay3.equals("Purple2")) {
//                        Bay1Boot.setPosition(ready);
//                        Bay2Boot.setPosition(ready);
//
//
//                        Bay3Boot.setPosition(shoot);
//                    }
//                }
//            }
//
//            if (ShootState.equals("Done")) {
//                Bay1Boot.setPosition(ready);
//                Bay2Boot.setPosition(ready);
//                Bay3Boot.setPosition(ready);
//////                            amshooting = false;
////                ShootState = "none";
////                return false;
//            }
//            telemetry.addData("shootstate", ShootState);
//            telemetry.addData("Pattern", pattern);
////            telemetry.addData("time", time2.seconds());
//            telemetry.update();
//            return false;
//
//        }
//    }


    public class ColorSense implements Action {
    private final NormalizedColorSensor bay11, bay12, bay21, bay22, bay31, bay32;
    private final DistanceSensor dist11, dist12, dist21, dist22, dist31, dist32;
    private final Limelight3A Limelight;
    private boolean initialized = false;
    private ElapsedTime timer;


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
                pattern = "none";
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
            NormalizedRGBA colors11 = bay11.getNormalizedColors();
            NormalizedRGBA colors12 = bay12.getNormalizedColors();
            double avgBlue1 = (colors11.blue + colors12.blue) / 2.0;
            double avgGreen1 = (colors11.green + colors12.green) / 2.0;
            double distance1 = Math.min(dist11.getDistance(DistanceUnit.CM), dist12.getDistance(DistanceUnit.CM));
            NormalizedRGBA colors21 = bay21.getNormalizedColors();
            NormalizedRGBA colors22 = bay22.getNormalizedColors();
            double avgBlue2 = (colors21.blue + colors22.blue) / 2.0;
            double avgGreen2 = (colors21.green + colors22.green) / 2.0;
            double distance2 = Math.min(dist21.getDistance(DistanceUnit.CM), dist22.getDistance(DistanceUnit.CM));
            NormalizedRGBA colors31 = bay31.getNormalizedColors();
            NormalizedRGBA colors32 = bay32.getNormalizedColors();
            double avgBlue3 = (colors31.blue + colors32.blue) / 2.0;
            double avgGreen3 = (colors31.green + colors32.green) / 2.0;
            double distance3 = Math.min(dist31.getDistance(DistanceUnit.CM), dist32.getDistance(DistanceUnit.CM));
            if (distance1 < 3) {
                if (avgBlue1 > avgGreen1) {
                   ColorBay1 = 1;
                } else if (avgGreen1 > avgBlue1){
                    ColorBay1 = 2;
                } else {
                    ColorBay1 = 0;
                }
            }
            if (distance2 < 3) {
                if (avgBlue2 > avgGreen2) {
                    ColorBay2 = 1;
                } else if (avgGreen2 > avgBlue2){
                    ColorBay2 = 2;//bay 2 is stipud
                } else {
                    ColorBay2 = 0;
                }
            }
            if (distance3 < 10) {
                if (avgBlue3 > avgGreen3) {
                    ColorBay3 = 1;
                } else if (avgGreen3 > avgBlue3){
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




