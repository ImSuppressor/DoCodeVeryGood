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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

@Autonomous(name="RedDepot", preselectTeleOp = "Drive26")
public class RedDepot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        pattern = "none";

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-64, -7, 45));

        //TODO:Init
        Limelight3A Limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Limelight.pipelineSwitch(0);
        Limelight.setPollRateHz(50);
        Limelight.start();

//        ColorSensor colorBay11 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorBay1.1");
//        ColorSensor colorBay12 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorBay1.2");
////        DistanceSensor colorBay12_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.2");
////        DistanceSensor colorBay11_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.1");
//        ColorSensor Bay21 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.1");
//        ColorSensor Bay22 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.2");
////        DistanceSensor Bay22_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.2");
////        DistanceSensor Bay21_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.1");
//        ColorSensor Bay31 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.1");
//        ColorSensor Bay32 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.2");
////        DistanceSensor Bay32_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.2");
////        DistanceSensor Bay31_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.1");
//        ElapsedTime time2;

        ElapsedTime time2;


        //TODO:Init Everything cracka
        Action Detect = drive.actionBuilder(new Pose2d(64, -7, 0))//move to park
//                .stopAndAdd(new Setpositionforservo(Server,1))
//                .stopAndAdd(new BlueDepot.ShootBall("Detect"))
                .waitSeconds(30)


                .build();
        Action Score = drive.actionBuilder((new Pose2d(55, 54, 45)))// scoreing pos
                .strafeToLinearHeading(new Vector2d(63, 32), 0)
                .build();
        Action lineUpONE = drive.actionBuilder(new Pose2d(55,54,0))
                .strafeToLinearHeading(new Vector2d(0, 40), 0)
                .build();
        Action lineONE = drive.actionBuilder(new Pose2d(63, 32, 0))
                .strafeToLinearHeading(new Vector2d(48, 0), 0)
                .build();
        Action shootLineONe = drive.actionBuilder(new Pose2d(48, 0, 45))
                .strafeToLinearHeading(new Vector2d(62, 32), 0)
                .build();
        Action LIneTWO = drive.actionBuilder(new Pose2d(62, 32, 0))
                .strafeToLinearHeading(new Vector2d(48, 12.5), 0)
                .build();
        Action shooLIneTWo = drive.actionBuilder(new Pose2d(48, 12.5, 45))
                .strafeToLinearHeading(new Vector2d(62, 32), 0)
                .build();
        Action LIentree = drive.actionBuilder(new Pose2d(-62, 32, 0))//
                .strafeToLinearHeading(new Vector2d(48, 36), 0)
                .build();
        Action ShootLINETHREe = drive.actionBuilder(new Pose2d(48, -36, 45))
                .strafeToLinearHeading(new Vector2d(-63, 32), 0)
                .build();
        Action PAAAAAAAAAAAAAARRRRRRRRRRRRRRRRRRK = drive.actionBuilder(new Pose2d(-63, 32, 0))
                .strafeToLinearHeading(new Vector2d(-40, 0), 0)
                .build();

        Action one = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
                .stopAndAdd(new RedDepot.ColorSense("off"))
                .waitSeconds(1)
                .stopAndAdd(new RedDepot.ShootBall("Shoot1"))
                .waitSeconds(1)

                .build();
        Action two = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
                .stopAndAdd(new RedDepot.ShootBall("Shoot2"))
                .waitSeconds(1)

                .build();
        Action three = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
                .stopAndAdd(new RedDepot.ShootBall("Shoot3"))
                .waitSeconds(1)

                .build();
        Action reset = drive.actionBuilder(new Pose2d(-64, -7, 0))
                .stopAndAdd(new RedDepot.ShootBall("Done"))
                .waitSeconds(4)
                .build();

        /// RUN DA CODE NOW CRACKA
        waitForStart();
        Actions.runBlocking(new SequentialAction(//place spec 1
                Detect,
                one,
                two,
                three,
                reset

        ));
//        Actions.runBlocking(new ParallelAction(//go to spike 1));
//                lineUpONE,
        // turn on da intake here i think?
//                lineONE
//
//        ));
//        Actions.runBlocking(new ParallelAction(//shoots line one
//                Detect,
//                one,
//                shootLineONe,
//                two,
//                three,
//                reset
//        ));



    }
    public class Setpositionforservo implements Action {
        Servo servo;
        double position;

        public Setpositionforservo(Servo servo, double position) {
            this.servo = servo;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }

    public class ShootBall implements Action {
        Servo Bay1Boot = hardwareMap.get(Servo.class, "Boot1");
        Servo Bay2Boot = hardwareMap.get(Servo.class, "Boot2");
        Servo Bay3Boot = hardwareMap.get(Servo.class, "Boot3");
        double shoot;
        double ready;
        double cycle;
        String ShootState;

        public ShootBall(String ShootState) {
            this.ShootState = ShootState;
            this.shoot = .6;
            this.ready = .95;
            this.cycle = 1;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (pattern.equals("PPG")) {
                if (ShootState.equals("Shoot1")) {
                    telemetry.addLine("ball1");
                    if (ColorBay1.equals("Purple1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (ShootState.equals("Shoot2")) {
                    telemetry.addLine("ball2");

                    if (ColorBay1.equals("Purple2")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (ShootState.equals("Shoot3")) {
                    telemetry.addLine("ball3");
                    if (ColorBay1.equals("Green1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Green1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Green1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
            }
            if (pattern.equals("PGP")) {
                if (ShootState.equals("Shoot1")) {

                    if (ColorBay1.equals("Purple1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (ShootState.equals("Shoot2")) {

                    if (ColorBay1.equals("Green1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Green1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Green1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (ShootState.equals("Shoot3")) {

                    if (ColorBay1.equals("Purple2")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
            }


            if (pattern.equals("GPP")) {
                if (ShootState.equals("Shoot1")) {
                    if (ColorBay1.equals("Green1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Green1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Green1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (ShootState.equals("Shoot2")) {

                    if (ColorBay1.equals("Purple1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (ShootState.equals("Shoot3")) {
                    if (ColorBay1.equals("Purple2")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
            }

            if (ShootState.equals("Done")) {
                Bay1Boot.setPosition(ready);
                Bay2Boot.setPosition(ready);
                Bay3Boot.setPosition(ready);
////                            amshooting = false;
//                ShootState = "none";
//                return false;
            }
            telemetry.addData("shootstate", ShootState);
            telemetry.addData("Pattern", pattern);
//            telemetry.addData("time", time2.seconds());
            telemetry.update();
            return false;

        }
    }


    public class ColorSense implements Action {
        String color;
        ColorSensor colorBay11 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay1.1");
        ColorSensor colorBay12 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay1.2");
        DistanceSensor colorBay12_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay1.2");
        DistanceSensor colorBay11_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay1.1");
        ColorSensor Bay21 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.1");
        ColorSensor Bay22 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.2");
        DistanceSensor Bay22_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.2");
        DistanceSensor Bay21_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.1");
        ColorSensor Bay31 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.1");
        ColorSensor Bay32 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.2");
        DistanceSensor Bay32_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.2");
        DistanceSensor Bay31_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.1");
        Limelight3A Limelight = hardwareMap.get(Limelight3A.class, "limelight");


        public ColorSense(String color) {
            this.color = color;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (opModeIsActive()) {
//                    if (!color.equals("on")) {
//                        return false;
//                    }
            if (color.equals("on")) {
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
                if ((((NormalizedColorSensor) colorBay11).getNormalizedColors().blue + ((NormalizedColorSensor) colorBay12).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) colorBay11).getNormalizedColors().green + ((NormalizedColorSensor) colorBay12).getNormalizedColors().green) / 2 && (colorBay12_DistanceSensor.getDistance(DistanceUnit.CM) <= 5 || colorBay11_DistanceSensor.getDistance(DistanceUnit.CM) <= 5)) {
                    if (!ColorBay3.equals("Purple1") && !ColorBay2.equals("Purple1")) {
                        ColorBay1 = "Purple1";
                    } else if ((ColorBay3.equals("Purple1") || ColorBay2.equals("Purple1")) && !ColorBay3.equals("Purple2") && !ColorBay2.equals("Purple2")) {
                        ColorBay1 = "Purple2";
                    } else if ((ColorBay3.equals("Purple1") || ColorBay2.equals("Purple1")) && (ColorBay3.equals("Purple2") || ColorBay2.equals("Purple2"))) {
                        ColorBay1 = "Purple3";
                    }
                    telemetry.addLine("Bay 1 Purple");
                } else if ((((NormalizedColorSensor) colorBay11).getNormalizedColors().green + ((NormalizedColorSensor) colorBay12).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) colorBay11).getNormalizedColors().blue + ((NormalizedColorSensor) colorBay12).getNormalizedColors().blue) / 2 && (colorBay11_DistanceSensor.getDistance(DistanceUnit.CM) <= 5 || colorBay12_DistanceSensor.getDistance(DistanceUnit.CM) <= 5)) {
                    if (!ColorBay3.equals("Green1") && !ColorBay2.equals("Green1")) {
                        ColorBay1 = "Green1";
                    } else if ((ColorBay3.equals("Green1") || ColorBay2.equals("Green1")) && !ColorBay3.equals("Green2") && !ColorBay2.equals("Green2")) {
                        ColorBay1 = "Green2";
                    } else if ((ColorBay3.equals("Green1") || ColorBay2.equals("Green1")) && (ColorBay3.equals("Green2") || ColorBay2.equals("Green2"))) {
                        ColorBay1 = "Green3";
                    }
                    telemetry.addLine("Bay 1 Green");
                } else {
                    ColorBay1 = "Empty";
                    telemetry.addLine("Bay 1 Empty");
                }
                if ((((NormalizedColorSensor) Bay21).getNormalizedColors().blue + ((NormalizedColorSensor) Bay22).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) Bay21).getNormalizedColors().green + ((NormalizedColorSensor) Bay22).getNormalizedColors().green) / 2 && (Bay22_DistanceSensor.getDistance(DistanceUnit.CM) <= 5 || Bay21_DistanceSensor.getDistance(DistanceUnit.CM) <= 5)) {
                    if (!ColorBay3.equals("Purple1") && !ColorBay1.equals("Purple1")) {
                        ColorBay2 = "Purple1";
                    } else if ((ColorBay3.equals("Purple1") || ColorBay1.equals("Purple1")) && !ColorBay3.equals("Purple2") && !ColorBay1.equals("Purple2")) {
                        ColorBay2 = "Purple2";
                    } else if ((ColorBay3.equals("Purple1") || ColorBay1.equals("Purple1")) && (ColorBay3.equals("Purple2") || ColorBay1.equals("Purple2"))) {
                        ColorBay2 = "Purple3";
                    }
                    telemetry.addLine("Bay 2 Purple");
                } else if ((((NormalizedColorSensor) Bay21).getNormalizedColors().green + ((NormalizedColorSensor) Bay22).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) Bay21).getNormalizedColors().blue + ((NormalizedColorSensor) Bay22).getNormalizedColors().blue) / 2 && (Bay21_DistanceSensor.getDistance(DistanceUnit.CM) <= 5 || Bay22_DistanceSensor.getDistance(DistanceUnit.CM) <= 5)) {
                    if (!ColorBay3.equals("Green1") && !ColorBay1.equals("Green1")) {
                        ColorBay2 = "Green1";
                    } else if ((ColorBay3.equals("Green1") || ColorBay1.equals("Green1")) && !ColorBay3.equals("Green2") && !ColorBay1.equals("Green2")) {
                        ColorBay2 = "Green2";
                    } else if ((ColorBay3.equals("Green1") || ColorBay1.equals("Green1")) && (ColorBay3.equals("Green2") || ColorBay1.equals("Green2"))) {
                        ColorBay2 = "Green3";
                    }
                    telemetry.addLine("Bay 2 Green");
                } else {
                    ColorBay2 = "Empty";
                    telemetry.addLine("Bay 2 Empty");
                }
                if ((((NormalizedColorSensor) Bay31).getNormalizedColors().blue + ((NormalizedColorSensor) Bay32).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) Bay31).getNormalizedColors().green + ((NormalizedColorSensor) Bay32).getNormalizedColors().green) / 2 && (Bay32_DistanceSensor.getDistance(DistanceUnit.CM) <= 10 || Bay31_DistanceSensor.getDistance(DistanceUnit.CM) <= 10)) {
                    if (!ColorBay1.equals("Purple1") && !ColorBay2.equals("Purple1")) {
                        ColorBay3 = "Purple1";
                    } else if ((ColorBay1.equals("Purple1") || ColorBay2.equals("Purple1")) && !ColorBay1.equals("Purple2") && !ColorBay2.equals("Purple2")) {
                        ColorBay3 = "Purple2";
                    } else if ((ColorBay1.equals("Purple1") || ColorBay2.equals("Purple1")) && (ColorBay1.equals("Purple2") || ColorBay2.equals("Purple2"))) {
                        ColorBay3 = "Purple3";
                    }
                    telemetry.addLine("Bay 3 Purple");
                } else if ((((NormalizedColorSensor) Bay31).getNormalizedColors().green + ((NormalizedColorSensor) Bay32).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) Bay31).getNormalizedColors().blue + ((NormalizedColorSensor) Bay32).getNormalizedColors().blue) / 2 && (Bay31_DistanceSensor.getDistance(DistanceUnit.CM) <= 10 || Bay32_DistanceSensor.getDistance(DistanceUnit.CM) <= 10)) {
                    if (!ColorBay1.equals("Green1") && !ColorBay2.equals("Green1")) {
                        ColorBay3 = "Green1";
                    } else if ((ColorBay1.equals("Green1") || ColorBay2.equals("Green1")) && !ColorBay1.equals("Green2") && !ColorBay2.equals("Green2")) {
                        ColorBay3 = "Green2";
                    } else if ((ColorBay1.equals("Green1") || ColorBay2.equals("Green1")) && (ColorBay1.equals("Green2") || ColorBay2.equals("Green2"))) {
                        ColorBay3 = "Green3";
                    }
                    telemetry.addLine("Bay 3 Green");
                } else {
                    ColorBay3 = "Empty";
                    telemetry.addLine("Bay 3 Empty");
                }
                telemetry.addData("Bay 3", ColorBay3);
                telemetry.addData("Bay 2", ColorBay2);
                telemetry.addData("Bay 1", ColorBay1);
                telemetry.addData("colorsense", color);
                telemetry.update();
            }


            return false;
        }


    }
}