package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "Webcam AprilTag Detector", group = "Vision")
public class webCamAprilTagDetector extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Initialize AprilTag detection
        initAprilTag();

        telemetry.addLine("AprilTag ready!");
        telemetry.addLine("Press START to begin detecting...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            detectAprilTags();
            sleep(50);
        }

        visionPortal.close();
    }

    // 🔹 Initialize AprilTag and Vision Portal
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    // 🔹 Method that checks if any AprilTags are visible
    private void detectAprilTags() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.size() > 0) {
            AprilTagDetection tag = detections.get(0); // read first detected tag
            telemetry.addData("Detected Tag ID", tag.id);
            telemetry.addData("Range (in)", tag.ftcPose.range);
            telemetry.addData("Bearing (deg)", tag.ftcPose.bearing);
            telemetry.addData("Yaw (deg)", tag.ftcPose.yaw);
            telemetry.update();
        } else {
            telemetry.addLine("No AprilTags detected");
        }
        telemetry.update();
    }
}
