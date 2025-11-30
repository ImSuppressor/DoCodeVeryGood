package org.firstinspires.ftc.teamcode;


// RR-specific imports

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.AtGoalToRed;





@Config
@Autonomous(name = "FrontOfFieldToRed", group = "Autonomous")
public class FrontOfFieldToRed extends LinearOpMode {


    DcMotor lf = null;
    DcMotor lb = null;
    DcMotor rf = null;
    DcMotor rb = null;

    DcMotor transfer =null;

    DcMotor launcher = null;

    // lift class
    private boolean initialized = false;

    public class warmupLaunch implements InstantFunction{
        @Override
        public void run(){
            launcher.setPower(-0.7);
        }
    }
    public class stopLauncher implements InstantFunction{
        @Override
        public void run(){
            launcher.setPower(0);
        }
    }

    public class transferArtifact implements InstantFunction{
        @Override
        public void run(){
            //   transfer.setPower(1);
            //  sleep(200);
            //    transfer.setPower(0);
        }
    }

    public class Shoot implements InstantFunction{
        @Override
        public void run(){
            transfer.setPower(-1);
            sleep(50);
            transfer.setPower(1);
            sleep(600);
            transfer.setPower(0);
        }
    }



    public void runOpMode() {
        transfer = hardwareMap.dcMotor.get("transfer");
        launcher = hardwareMap.dcMotor.get("launcher");
        Pose2d beginPose = new Pose2d(new Vector2d(56,12), Math.toRadians(0));
        //this pose assumes the robot starts with the intake facing away from the goal. the shooter will be facing away from the goal

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        //creating RR path

        // actionBuilder builds from the drive steps passed to it
        //this path moves backwards and turns
        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(new transferArtifact())
                .stopAndAdd(new warmupLaunch())
                .lineToX(-18)
                .turn(Math.toRadians(130))
                .stopAndAdd(new Shoot())
                .strafeTo(new Vector2d(-30,25))
                .stopAndAdd(new transferArtifact())
                .stopAndAdd(new Shoot())
                .waitSeconds(2)
                .stopAndAdd(new stopLauncher())
                .strafeTo(new Vector2d(-18,12))
                .turn(Math.toRadians(50))
                .lineToX(56)
                .build();

        Action closeshot = drive.actionBuilder((beginPose))
                .lineToX(34)
                .splineTo(new Vector2d(-20, 20), Math.toRadians(-90))
                .turn(Math.toRadians(45))
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(36,20))
                .strafeTo(new Vector2d(36,56))
                .strafeTo(new Vector2d(36,20))
                .splineTo(new Vector2d(-20, 20), Math.toRadians(-90))
                .turn(Math.toRadians(45))
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(56,20))
                        .build();

        Action farshot = drive.actionBuilder((beginPose))
                .lineToX(38)
                .turn(Math.toRadians(180))
                .lineToX(50)
                .turn(Math.toRadians(-20))
                .splineTo(new Vector2d(38, 25), Math.toRadians(90))
                .strafeTo(new Vector2d(38,56))
                .strafeTo(new Vector2d(38,20))
                .splineTo(new Vector2d(50,12),Math.toRadians(0))
                .turn(Math.toRadians(-20))
                .turn(Math.toRadians(20))
                .lineToX(38)
                .build();


        Actions.runBlocking(new SequentialAction(path));








    }

}

