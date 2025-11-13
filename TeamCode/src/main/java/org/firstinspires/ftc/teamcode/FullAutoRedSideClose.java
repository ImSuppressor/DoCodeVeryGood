

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "FULL_AUTO_RED_ClOSE_PIXEL", group = "Autonomous")
public class FullAutoRedSideClose extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx intakemotor = null;
    private ElapsedTime kickTimer = new ElapsedTime();
    private double kickCycleTime = 2.5;
    private Servo kicker;
    private DcMotorEx outtakemotorright = null;
    private DcMotorEx outtakemotorleft = null;
    private Servo outtakeservo = null;
    private double home = 0, kick = 0.8;

    @Override
    public void runOpMode() {
        outtakemotorright = hardwareMap.get(DcMotorEx.class, "outtakemotorright");
        outtakeservo = hardwareMap.get(Servo.class, "outtakeservo");
//        intakemotortwo = hardwareMap.get(DcMotorEx.class, "intakemotortwo");
        outtakemotorleft = hardwareMap.get(DcMotorEx.class,"outtakemotorleft");
        intakemotor = hardwareMap.get(DcMotorEx.class,"intakemotor");
        kicker = hardwareMap.get(Servo.class,"kickservo");

        //TODO: instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        initAprilTag();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


        kicker.setPosition(0);
        outtakeservo.setPosition(0.475);
        kickTimer.reset();
        waitForStart();
        visionPortal.close();

        // First run
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                //.lineToYSplineHeading(24, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, 0), Math.toRadians(90))
                .turn(Math.toRadians(-45));
        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);

        //TODO: launch code here
        outtakemotorright.setPower(-0.4);
        outtakemotorleft.setPower(0.4);
        kickAuto(1); // 1st ball
        intakemotor.setPower(1);// Push 2nd ball forward
        kickAuto(2); // 2nd ball

        //2nd run
//        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
//                .splineToConstantHeading(new Vector2d(48, 0), Math.toRadians(90))
//                .turn(Math.toRadians(135));
//        trajectoryActionChosen = goToIntake.build();
//        Actions.runBlocking(trajectoryActionChosen);

        intakemotor.setPower(0.75);

        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
                .turn(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(48, 24), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(48, 0), Math.toRadians(0))
                .turn(Math.toRadians(-135));
        trajectoryActionChosen = goToLaunchSpot2.build();
        Actions.runBlocking(trajectoryActionChosen);


        intakemotor.setPower(0);
//        outtakemotorright.setPower(-0.4);
//        outtakemotorleft.setPower(0.4);
//
//        kickAuto(1); //Kick 1st ball
//        intakemotor.setPower(0.75);// load 2nd ball
//        kickAuto(2); //kick 2nd ball

        if (isStopRequested()) {
            return;
        }

    }
    private void kickAuto(int ballNumber) {
        kicker.setPosition(kick);
        outtakeservo.setPosition(0.475);
        waitForTime(kickCycleTime);
        kicker.setPosition(0);

        if (1 == ballNumber) {
            //So kicker has time to go back to position 0
            waitForTime(kickCycleTime);
        }
    }
    private void waitForTime(double waitTime) {
        kickTimer.reset();
        while (kickTimer.seconds() < waitTime) {
            //do nothing, just wait

            //telemetry.addData("waiting to kick: ", kickTimer.time());
        }
    }
    private Pose2d getCurrentPos(MecanumDrive drive) {
        return drive.localizer.getPose();
    }










    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }



    private void telemetryAprilTag () {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}