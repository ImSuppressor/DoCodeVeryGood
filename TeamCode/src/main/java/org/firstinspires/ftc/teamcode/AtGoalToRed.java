package org.firstinspires.ftc.teamcode;


// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;





@Config
@Autonomous(name = "AtGoalToRed", group = "Autonomous")
public class AtGoalToRed extends LinearOpMode {


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
        Pose2d beginPose = new Pose2d(new Vector2d(-52,49), Math.toRadians(-45));
        //this pose assumes the robot starts with the intake facing away from the goal. the shooter will be facing away from the goal

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        //creating RR path

        // actionBuilder builds from the drive steps passed to it
        //this path moves backwards and turns
        Action path = drive.actionBuilder(beginPose)
                .stopAndAdd(new warmupLaunch())
                .stopAndAdd(new transferArtifact())
                .lineToX(-25)
                .turn(Math.toRadians(180))
                .stopAndAdd(new Shoot())
                .stopAndAdd(new stopLauncher())
                .build();

        Action path2 = drive.actionBuilder((beginPose))
                .stopAndAdd(new warmupLaunch())
                .waitSeconds(5)
//                .stopAndAdd(TransferArtifact())
                .waitSeconds(1)
                .stopAndAdd(new stopLauncher())
                .build();


        Actions.runBlocking(new SequentialAction(path));








    }

}







