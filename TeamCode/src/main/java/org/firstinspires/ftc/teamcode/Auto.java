package org.firstinspires.ftc.teamcode;


// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
@Autonomous(name = "AyoAuto", group = "Autonomous")
public abstract class Auto extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(Auto.class);
    DcMotor lf = null;
    DcMotor lb = null;
    DcMotor rf = null;
    DcMotor rb = null;
    Servo leftfeeder = null;
    Servo rightfeeder = null;
    DcMotor launcher = null;

    // lift class
    public class Drivetrain {
// Drivetrain Settings
        public void DriveTrain() {
            lf = hardwareMap.get(DcMotorEx.class, "left_front_drive");
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lf.setDirection(DcMotorSimple.Direction.FORWARD);

            lb = hardwareMap.get(DcMotorEx.class, "left_back_drive");
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setDirection(DcMotorSimple.Direction.REVERSE);

            rf = hardwareMap.get(DcMotorEx.class, "right_front_drive");
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setDirection(DcMotorSimple.Direction.REVERSE);

            rb = hardwareMap.get(DcMotorEx.class, "right_back_drive");
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setDirection(DcMotorSimple.Direction.REVERSE);

            leftfeeder = hardwareMap.get(Servo.class, "leftfeeder");
            leftfeeder.setDirection(Servo.Direction.FORWARD);

            rightfeeder = hardwareMap.get(Servo.class, "rightfeeder");
            rightfeeder.setDirection(Servo.Direction.FORWARD);

            launcher = hardwareMap.get(DcMotor.class, "launcher");
            launcher.setDirection(DcMotor.Direction.FORWARD);
        }

        // within the Lift class

            private boolean initialized = false;



        // within the Claw class
        public class WarmupLaunch implements Action {

            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setPower(1);
                return false;
            }
        }
        public Action WarmupLaunch() {
            return new WarmupLaunch();
        }

        public class StopLaunch implements Action {

            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setPower(0);
                return false;
            }
        }
        public Action StopLaunch() {
            return new StopLaunch();
        }

//        public class TransferArtifact implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftfeeder.setPosition(1.0);
//                rightfeeder.setPosition(1.0);
//                return false;
//            }
//        }
//        public Action TransferArtifact() {
//            return new TransferArtifact();
//        }

        public void runOpMode() {
            Pose2d beginPose = new Pose2d(new Vector2d(72,-16), Math.toRadians(270));

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            //creating RR path

            // actionBuilder builds from the drive steps passed to it
            Action path = drive.actionBuilder(beginPose)
                    .lineToX(50)
                    .turn(Math.toRadians(90))
                    .lineToY(30)
                    .stopAndAdd(WarmupLaunch())
                    .turn(Math.toRadians(90))
                    .lineToX(-40)
                    .turn(Math.toRadians(-240))
//                    .stopAndAdd(TransferArtifact())
                    .stopAndAdd(StopLaunch())
                    .build();

            Action Shooter = drive.actionBuilder((beginPose))
                    .stopAndAdd(WarmupLaunch())
                    .waitSeconds(5)
                    //.stopAndAdd(TransferArtifact())
                    .waitSeconds(1)
                    .stopAndAdd(StopLaunch())
                    .build();


            Actions.runBlocking(new SequentialAction(path));








        }

    }







}