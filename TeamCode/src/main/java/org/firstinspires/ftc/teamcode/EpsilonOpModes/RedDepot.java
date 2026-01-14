package org.firstinspires.ftc.teamcode.EpsilonOpModes;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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


        waitForStart();
        Actions.runBlocking(new ParallelAction(//place spec 1
                Detect

        ));


    }
}