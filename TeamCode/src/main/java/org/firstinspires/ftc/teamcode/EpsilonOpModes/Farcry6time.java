package org.firstinspires.ftc.teamcode.EpsilonOpModes;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;

@Autonomous(name="Farcry6time", preselectTeleOp = "Drive26")
public class Farcry6time extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        pattern = "none";

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -68, 45));

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
        Action Detect = drive.actionBuilder(new Pose2d(0, -68, 0))//move to park
//                .stopAndAdd(new Setpositionforservo(Server,1))
//                .stopAndAdd(new BlueDepot.ShootBall("Detect"))
                .waitSeconds(30)
                .build();
        Action ScorePRE = drive.actionBuilder(new Pose2d(0,-68,0))
                .strafeToLinearHeading(new Vector2d(48, 36), 0)
                .build();
        Action Spike3 = drive.actionBuilder(new Pose2d(48,36,0))
                .strafeToLinearHeading(new Vector2d(0, -68), 0)
                .build();
        Action ShootSpike3 =drive.actionBuilder(new Pose2d(0,-68,0))
                .strafeToLinearHeading(new Vector2d(50, 60), 0)
                .build();
//        Action GATE = drive.actionBuilder(new Pose2d(50,60,0))
//                .strafeToLinearHeading()
//                .build();


    }
}






























































































































