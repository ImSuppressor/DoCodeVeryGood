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
@Autonomous(name = "FrontOfFieldToRed", group = "Autonomous")
public class FrontOfFieldToRed extends LinearOpMode {


    DcMotor lf = null;
    DcMotor lb = null;
    DcMotor rf = null;
    DcMotor rb = null;
    Servo leftfeeder = null;
    Servo rightfeeder = null;
    DcMotor launcher = null;

    // lift class
    private boolean initialized = false;



    // within the Claw class
    public class WarmupLaunch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            launcher.setPower(1);
            return false;
        }
    }
    public Action Launch() {
        return new WarmupLaunch();
    }

    public class StopLaunch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            launcher.setPower(0);
            return false;
        }
    }
    public Action StopLaunch() {
        return new StopLaunch();
    }

    public class TransferArtifact implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftfeeder.setPosition(1.0);
            rightfeeder.setPosition(1.0);
            return false;
        }
    }

    int apriltag;

    public Action PositionAprilTag() {
        return new StopLaunch();
    }

    public class Position implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(apriltag == 43){
                return true;
            } else if(apriltag > 43){
                //move forward
            } else if(apriltag < 43){
                //move backward
            }
            return false;
        }
    }
    public Action TransferArtifact() {
        return new TransferArtifact();
    }

    public class Launch implements InstantFunction{
        @Override
        public void run(){
            launcher.setPower(1);
        }
    }




    public void runOpMode() {
        Pose2d beginPose = new Pose2d(new Vector2d(56,12), Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        //creating RR path

        // actionBuilder builds from the drive steps passed to it
        Action path = drive.actionBuilder(beginPose)
                .lineToX(-27)
                .turn(Math.toRadians(130))
                .build();

        Action path2 = drive.actionBuilder((beginPose))
                .stopAndAdd(Launch())
                .waitSeconds(5)
                .stopAndAdd(TransferArtifact())
                .waitSeconds(1)
                .stopAndAdd(PositionAprilTag())
                .stopAndAdd(StopLaunch())
                .build();


        Actions.runBlocking(new SequentialAction(path));








    }

}
