package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BlueDepot", preselectTeleOp = "Drive26")
public class BlueDepot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-64, -7, 0));

        //TODO KYS NOW

        ColorSensor colorBay11 = hardwareMap.colorSensor.get("colorBay1.1");
        ColorSensor colorBay12 = hardwareMap.colorSensor.get("colorBay1.2");
        DistanceSensor colorBay12_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.2");
        DistanceSensor colorBay11_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("colorBay1.1");
        ColorSensor Bay21 = hardwareMap.colorSensor.get("Bay2.1");
        ColorSensor Bay22 = hardwareMap.colorSensor.get("Bay2.2");
        DistanceSensor Bay22_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.2");
        DistanceSensor Bay21_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay2.1");
        ColorSensor Bay31 = hardwareMap.colorSensor.get("Bay3.1");
        ColorSensor Bay32 = hardwareMap.colorSensor.get("Bay3.2");
        DistanceSensor Bay32_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.2");
        DistanceSensor Bay31_DistanceSensor = (DistanceSensor) hardwareMap.colorSensor.get("Bay3.1");
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
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }

    public class ShootBall implements Action {
        Servo Bay1Boot;
        Servo Bay2Boot;
        Servo Bay3Boot;
//        boolean Booter1 = false;
//        boolean Booter2 = false;
//        boolean Booter3 = false;
        double shoot;
        double ready;
        double Shoot;
        String ShootState;
        String pattern;
        String ColorBay1;
        String ColorBay2;
        String ColorBay3;

        private ColorSensor colorBay11;
        private ColorSensor colorBay12;
        private DistanceSensor colorBay12_DistanceSensor;
        private DistanceSensor colorBay11_DistanceSensor;
        private ColorSensor Bay21;
        private ColorSensor Bay22;
        private DistanceSensor Bay22_DistanceSensor;
        private DistanceSensor Bay21_DistanceSensor;
        private ColorSensor Bay31;
        private ColorSensor Bay32;
        private DistanceSensor Bay32_DistanceSensor;
        private DistanceSensor Bay31_DistanceSensor;
//        ElapsedTime time2;

        public ShootBall(String ShootState) {
            this.shoot = 1;
            this.ready = 0;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if ((ShootState.equals("Detect"))){
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
            }
            if (ShootState.equals("Shoot")) {
                if(pattern.equals("PPG")){
                    if (ColorBay1.equals("Purple1")) {
//                        time2.reset();
//                        Booter1 = true;
                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
//                        time2.reset();
//                        Booter2 = true;
                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
//                        time2.reset();
//                        Booter3 = true;
                        Bay3Boot.setPosition(shoot);
                    }


                }
                else if (pattern.equals("PGP")) {

                }
                else if (pattern.equals("GPP")) {

                }
                else {

                }
            }


            return false;
        }
    }
}
