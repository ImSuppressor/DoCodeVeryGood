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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "FULL_AUTO_BLUE_PIXEL", group = "Autonomous")
public class FullAutoBlueSide extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx intake = null;
    private DcMotorEx conveyerBelt = null;
    private DcMotorEx outtake = null;


    @Override
    public void runOpMode() {

        //TODO: instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        initAprilTag();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


        waitForStart();
        visionPortal.close();


        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                //.lineToYSplineHeading(24, Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, 0), Math.toRadians(0))
                .turn(Math.toRadians(45));
        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);

        //TODO: launch code here



        TrajectoryActionBuilder intake3Balls = drive.actionBuilder(getCurrentPos(drive))
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
                .turn(Math.toRadians(45))
                //facing left
                //intake start code here

                .splineToConstantHeading(new Vector2d(0, -24), Math.toRadians(0))
                .turn(Math.toRadians(-90));

        intake.setPower(1);// double type
        intake.setVelocity(10);// double type. ENCODER wire must be connected.
        //launched, collected 3
        //conveyer belt code here
        trajectoryActionChosen = intake3Balls.build();
        Actions.runBlocking(trajectoryActionChosen);


        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, 0), Math.toRadians(0))
                .turn(Math.toRadians(45));
        trajectoryActionChosen = goToLaunchSpot2.build();
        Actions.runBlocking(trajectoryActionChosen);



        if (isStopRequested()) {
            return;
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