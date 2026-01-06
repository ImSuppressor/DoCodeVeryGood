package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GlobalVar.pattern;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

@Autonomous(name="BlueDepot", preselectTeleOp = "Drive26")
public class BlueDepot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        pattern = "none";

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-64, -7, 0));

        //TODO:Init
        Limelight3A Limelight = hardwareMap.get(Limelight3A.class,"limelight");
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
        Action Detect = drive.actionBuilder(new Pose2d(-64, -7, 0))//move to park
//                .stopAndAdd(new Setpositionforservo(Server,1))
                .stopAndAdd(new ShootBall("Detect"))
                .waitSeconds(30)


                .build();




        waitForStart();
        Actions.runBlocking(new ParallelAction(//place spec 1
                Detect

        ));


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

    public class ShootBall implements Action  {
//        Servo Bay1Boot = hardwareMap.get(Servo.class,"Boot1");
//        Servo Bay2Boot = hardwareMap.get(Servo.class,"Boot2");
//        Servo Bay3Boot = hardwareMap.get(Servo.class,"Boot3");
        boolean Booter1 = false;
        boolean Booter2 = false;
        boolean Booter3 = false;
        double shoot;
        double ready;
        double cycle;
        String ShootState;
//        public Constants = new Constants;
        String ColorBay1 = "Empty";
        String ColorBay2 = "Empty";
        String ColorBay3 = "Empty";
        ColorSensor colorBay11 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorBay1.1");
        ColorSensor colorBay12 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorBay1.2");
        DistanceSensor colorBay12_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.2");
        DistanceSensor colorBay11_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.1");
        ColorSensor Bay21 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.1");
        ColorSensor Bay22 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.2");
        DistanceSensor Bay22_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.2");
        DistanceSensor Bay21_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.1");
        ColorSensor Bay31 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.1");
        ColorSensor Bay32 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.2");
        DistanceSensor Bay32_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.2");
        DistanceSensor Bay31_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.1");
//        ColorSensor colorBay11 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorBay1.1");
//        ColorSensor colorBay12 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorBay1.2");
//                DistanceSensor colorBay12_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.2");
//        DistanceSensor colorBay11_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.1");
//        ColorSensor Bay21 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.1");
//        ColorSensor Bay22 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay2.2");
//                DistanceSensor Bay22_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.2");
//        DistanceSensor Bay21_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.1");
//        ColorSensor Bay31 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.1");
//        ColorSensor Bay32 = (ColorSensor) hardwareMap.get(NormalizedColorSensor.class, "Bay3.2");
//        DistanceSensor Bay32_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.2");
//        DistanceSensor Bay31_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.1");

        Limelight3A Limelight = hardwareMap.get(Limelight3A.class,"limelight");

        ElapsedTime time2;
        boolean amshooting;

        public ShootBall(String ShootState) {
            this.time2 = new ElapsedTime();
            this.ShootState = ShootState;
            this.shoot = 1;
            this.ready = 0;
            this.cycle = 1;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    LLResult result = Limelight.getLatestResult();
                    for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()){
                    if (pattern.equals("none")) {
                        if (fiducial.getFiducialId() == 21){
                            pattern = "GPP";
                        }
                        else if (fiducial.getFiducialId() == 22) {
                            pattern = "PGP";
                        }
                        else if (fiducial.getFiducialId() == 23) {
                            pattern = "PPG";
                        }
                    }
                    telemetry.addData("paty",pattern);
                    }
                    if ((((NormalizedColorSensor) colorBay11).getNormalizedColors().blue + ((NormalizedColorSensor) colorBay12).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) colorBay11).getNormalizedColors().green + ((NormalizedColorSensor) colorBay12).getNormalizedColors().green) / 2 && (colorBay12_DistanceSensor.getDistance(DistanceUnit.CM) <= 3 || colorBay11_DistanceSensor.getDistance(DistanceUnit.CM) <= 3)) {
                        if (!ColorBay3.equals("Purple1") && !ColorBay2.equals("Purple1")) {
                            ColorBay1 = "Purple1";
                        } else if ((ColorBay3.equals("Purple1") || ColorBay2.equals("Purple1")) && !ColorBay3.equals("Purple2") && !ColorBay2.equals("Purple2")) {
                            ColorBay1 = "Purple2";
                        } else if ((ColorBay3.equals("Purple1") || ColorBay2.equals("Purple1")) && (ColorBay3.equals("Purple2") || ColorBay2.equals("Purple2"))) {
                            ColorBay1 = "Purple3";
                        }
                        telemetry.addLine("Bay 1 Purple");
                    } else if ((((NormalizedColorSensor) colorBay11).getNormalizedColors().green + ((NormalizedColorSensor) colorBay12).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) colorBay11).getNormalizedColors().blue + ((NormalizedColorSensor) colorBay12).getNormalizedColors().blue) / 2 && (colorBay11_DistanceSensor.getDistance(DistanceUnit.CM) <= 3 || colorBay12_DistanceSensor.getDistance(DistanceUnit.CM) <= 3)) {
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
                    if ((((NormalizedColorSensor) Bay21).getNormalizedColors().blue + ((NormalizedColorSensor) Bay22).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) Bay21).getNormalizedColors().green + ((NormalizedColorSensor) Bay22).getNormalizedColors().green) / 2 && (Bay22_DistanceSensor.getDistance(DistanceUnit.CM) <= 3 || Bay21_DistanceSensor.getDistance(DistanceUnit.CM) <= 3)) {
                        if (!ColorBay3.equals("Purple1") && !ColorBay1.equals("Purple1")) {
                            ColorBay2 = "Purple1";
                        } else if ((ColorBay3.equals("Purple1") || ColorBay1.equals("Purple1")) && !ColorBay3.equals("Purple2") && !ColorBay1.equals("Purple2")) {
                            ColorBay2 = "Purple2";
                        } else if ((ColorBay3.equals("Purple1") || ColorBay1.equals("Purple1")) && (ColorBay3.equals("Purple2") || ColorBay1.equals("Purple2"))) {
                            ColorBay2 = "Purple3";
                        }
                        telemetry.addLine("Bay 2 Purple");
                    } else if ((((NormalizedColorSensor) Bay21).getNormalizedColors().green + ((NormalizedColorSensor) Bay22).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) Bay21).getNormalizedColors().blue + ((NormalizedColorSensor) Bay22).getNormalizedColors().blue) / 2 && (Bay21_DistanceSensor.getDistance(DistanceUnit.CM) <= 3 || Bay22_DistanceSensor.getDistance(DistanceUnit.CM) <= 3)) {
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
                    telemetry.update();

                if (ShootState.equals("Shoot")) {
                    if (amshooting == false) {
                        time2.reset();
                    }
                    if (pattern.equals("PPG")) {
                        if (ColorBay1.equals("Purple1")) {
                        time2.reset();
                        Booter1 = true;
                        //    Bay1Boot.setPosition(shoot);
                        } else if (ColorBay2.equals("Purple1")) {
                        time2.reset();
                            Booter2 = true;
                        //    Bay2Boot.setPosition(shoot);
                        } else if (ColorBay3.equals("Purple1")) {
                        time2.reset();
                        Booter3 = true;
                        //    Bay3Boot.setPosition(shoot);
                        }
                        if (ColorBay1.equals("Purple2")) {
                            time2.reset();
                            Booter1 = true;
                            //    Bay1Boot.setPosition(shoot);
                        } else if (ColorBay2.equals("Purple2")) {
                            time2.reset();
                            Booter2 = true;
                            //    Bay2Boot.setPosition(shoot);
                        } else if (ColorBay3.equals("Purple2")) {
                            time2.reset();
                            Booter3 = true;
                            //    Bay3Boot.setPosition(shoot);
                        }
                        if (ColorBay1.equals("Green1")) {
                            time2.reset();
                            Booter1 = true;
                            //    Bay1Boot.setPosition(shoot);
                        } else if (ColorBay2.equals("Green1")) {
                            time2.reset();
                            Booter2 = true;
                            //    Bay2Boot.setPosition(shoot);
                        } else if (ColorBay3.equals("Green1")) {
                            time2.reset();
                            Booter3 = true;
                            //    Bay3Boot.setPosition(shoot);
                        }



                    }
//                    } else if (pattern.equals("PGP")) {
//
//                    } else if (pattern.equals("GPP")) {
//
//                    } else {
//                        ColorBay1 = "Empty";
//                        telemetry.addLine("Bay 1 Empty");
//                    }
//                    if ((((NormalizedColorSensor) Bay21).getNormalizedColors().blue + ((NormalizedColorSensor) Bay22).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) Bay21).getNormalizedColors().green + ((NormalizedColorSensor) Bay22).getNormalizedColors().green) / 2 && (Bay22_DistanceSensor.getDistance(DistanceUnit.CM) <= 3 || Bay21_DistanceSensor.getDistance(DistanceUnit.CM) <= 3)) {
//                        if (!ColorBay3.equals("Purple1") && !ColorBay1.equals("Purple1")) {
//                            ColorBay2 = "Purple1";
//                        } else if ((ColorBay3.equals("Purple1") || ColorBay1.equals("Purple1")) && !ColorBay3.equals("Purple2") && !ColorBay1.equals("Purple2")) {
//                            ColorBay2 = "Purple2";
//                        } else if ((ColorBay3.equals("Purple1") || ColorBay1.equals("Purple1")) && (ColorBay3.equals("Purple2") || ColorBay1.equals("Purple2"))) {
//                            ColorBay2 = "Purple3";
//                        }
//                        telemetry.addLine("Bay 2 Purple");
//                    } else if ((((NormalizedColorSensor) Bay21).getNormalizedColors().green + ((NormalizedColorSensor) Bay22).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) Bay21).getNormalizedColors().blue + ((NormalizedColorSensor) Bay22).getNormalizedColors().blue) / 2 && (Bay21_DistanceSensor.getDistance(DistanceUnit.CM) <= 3 || Bay22_DistanceSensor.getDistance(DistanceUnit.CM) <= 3)) {
//                        if (!ColorBay3.equals("Green1") && !ColorBay1.equals("Green1")) {
//                            ColorBay2 = "Green1";
//                        } else if ((ColorBay3.equals("Green1") || ColorBay1.equals("Green1")) && !ColorBay3.equals("Green2") && !ColorBay1.equals("Green2")) {
//                            ColorBay2 = "Green2";
//                        } else if ((ColorBay3.equals("Green1") || ColorBay1.equals("Green1")) && (ColorBay3.equals("Green2") || ColorBay1.equals("Green2"))) {
//                            ColorBay2 = "Green3";
//                        }
//                        telemetry.addLine("Bay 2 Green");
//                    } else {
//                        ColorBay2 = "Empty";
//                        telemetry.addLine("Bay 2 Empty");
//                    }
//                    if ((((NormalizedColorSensor) Bay31).getNormalizedColors().blue + ((NormalizedColorSensor) Bay32).getNormalizedColors().blue) / 2 > (((NormalizedColorSensor) Bay31).getNormalizedColors().green + ((NormalizedColorSensor) Bay32).getNormalizedColors().green) / 2 && (Bay32_DistanceSensor.getDistance(DistanceUnit.CM) <= 10 || Bay31_DistanceSensor.getDistance(DistanceUnit.CM) <= 10)) {
//                        if (!ColorBay1.equals("Purple1") && !ColorBay2.equals("Purple1")) {
//                            ColorBay3 = "Purple1";
//                        } else if ((ColorBay1.equals("Purple1") || ColorBay2.equals("Purple1")) && !ColorBay1.equals("Purple2") && !ColorBay2.equals("Purple2")) {
//                            ColorBay3 = "Purple2";
//                        } else if ((ColorBay1.equals("Purple1") || ColorBay2.equals("Purple1")) && (ColorBay1.equals("Purple2") || ColorBay2.equals("Purple2"))) {
//                            ColorBay3 = "Purple3";
//                        }
//                        telemetry.addLine("Bay 3 Purple");
//                    } else if ((((NormalizedColorSensor) Bay31).getNormalizedColors().green + ((NormalizedColorSensor) Bay32).getNormalizedColors().green) / 2 > (((NormalizedColorSensor) Bay31).getNormalizedColors().blue + ((NormalizedColorSensor) Bay32).getNormalizedColors().blue) / 2 && (Bay31_DistanceSensor.getDistance(DistanceUnit.CM) <= 10 || Bay32_DistanceSensor.getDistance(DistanceUnit.CM) <= 10)) {
//                        if (!ColorBay1.equals("Green1") && !ColorBay2.equals("Green1")) {
//                            ColorBay3 = "Green1";
//                        } else if ((ColorBay1.equals("Green1") || ColorBay2.equals("Green1")) && !ColorBay1.equals("Green2") && !ColorBay2.equals("Green2")) {
//                            ColorBay3 = "Green2";
//                        } else if ((ColorBay1.equals("Green1") || ColorBay2.equals("Green1")) && (ColorBay1.equals("Green2") || ColorBay2.equals("Green2"))) {
//                            ColorBay3 = "Green3";
//                        }
//                        telemetry.addLine("Bay 3 Green");
//                    } else {
//                        ColorBay3 = "Empty";
//                        telemetry.addLine("Bay 3 Empty");
//                    }
//                    telemetry.addData("Bay 3", ColorBay3);
//                    telemetry.addData("Bay 2", ColorBay2);
//                    telemetry.addData("Bay 1", ColorBay1);
//                    telemetry.update();
//
//                if (ShootState.equals("Shoot")) {
//                    if (pattern.equals("PPG")) {
//                        if (ColorBay1.equals("Purple1")) {
////                        time2.reset();
////                        Booter1 = true;
//                            Bay1Boot.setPosition(shoot);
//                        } else if (ColorBay2.equals("Purple1")) {
////                        time2.reset();
////                        Booter2 = true;
//                            Bay2Boot.setPosition(shoot);
//                        } else if (ColorBay3.equals("Purple1")) {
////                        time2.reset();
////                        Booter3 = true;
//                            Bay3Boot.setPosition(shoot);
//                        }
//
//                    }
////                    } else if (pattern.equals("PGP")) {
////
////                    } else if (pattern.equals("GPP")) {
////
////                    } else {
////
////                    }
                }
                }
            }


            return false;
        }
    }
}