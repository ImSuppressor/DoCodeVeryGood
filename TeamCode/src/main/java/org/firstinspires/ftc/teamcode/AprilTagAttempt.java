package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
public class AprilTagAttempt {





    @TeleOp(name = "AprilTagAttempt", group = "Concept")
    public class AprilTagShootTag24 extends LinearOpMode {

        private static final boolean USE_WEBCAM = true;

        private static final int TARGET_TAG_ID = 24;  // <-- your specific tag ID

        private AprilTagProcessor aprilTag;
        private VisionPortal visionPortal;

        private DcMotor shooter;

        private boolean hasShot = false; // ensure one shot per detection

        @Override
        public void runOpMode() {

            // --- Initialize Shooter ---
            shooter = hardwareMap.get(DcMotor.class, "shooter");
            shooter.setPower(0);

            // --- Initialize AprilTag ---
            initAprilTag();

            telemetry.addData(">", "Touch START to begin");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# Tags Detected", currentDetections.size());

                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addData("Tag ID", detection.id);

                    // Shoot only if the detected tag is 24
                    if (detection.id == TARGET_TAG_ID && !hasShot) {
                        telemetry.addData("Action", "Target tag detected! Shooting...");
                        telemetry.update();

                        shootOnce();

                        hasShot = true;
                        break; // stop checking other tags after shooting
                    }
                }

                telemetry.update();
                sleep(20);
            }

            visionPortal.close();
        }

        private void initAprilTag() {
            aprilTag = new AprilTagProcessor.Builder().build();

            VisionPortal.Builder builder = new VisionPortal.Builder();
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            }

            builder.addProcessor(aprilTag);
            visionPortal = builder.build();
        }

        private void shootOnce() {
            double power = 0.8;      // adjust for your shooter motor
            long durationMs = 1200;  // how long to spin motor

            shooter.setPower(power);
            sleep(durationMs);
            shooter.setPower(0);
            telemetry.addData("Action", "Shot fired!");
            telemetry.update();
        }
    }
}