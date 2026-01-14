package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GlobalVar.ColorBay1;
import static org.firstinspires.ftc.teamcode.GlobalVar.ColorBay2;
import static org.firstinspires.ftc.teamcode.GlobalVar.ColorBay3;
import static org.firstinspires.ftc.teamcode.GlobalVar.pattern;
import static org.firstinspires.ftc.teamcode.GlobalVar.team;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
@TeleOp(name = "Drive26")
public class Drive26 extends LinearOpMode {

//    private Servo pin;

    /**S
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -68, 45));
        double red_goalX = 72;////red is at 72, 72
        double red_goalY = 72;
        double blue_goalX = 0;
        double blue_goalY = -72;

//        pin = hardwareMap.get(Servo.class, "pin");
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("pattern", GlobalVar.pattern);


            while (opModeIsActive()) {
                // Put loop blocks here.
                drive.updatePoseEstimate();
                Pose2d sigma = drive.localizer.getPose();
                double currentX = sigma.position.x;
                double currentY = sigma.position.y;
                if (team.equals("red")) {
                    if (currentX < 0) {
                        double DisToGoalX = -currentX + 72;

                    }
                    if (currentX > 0) {
                        double DisToGoalX = currentX;

                    }
                    if (currentY < 0) {
                        double DisToGoalY = -currentY + 72;

                    }
                    if (currentY > 0) {
                        double DisToGoalY = currentY;

                    }
                }
                if (team.equals("blue")) {
                    if (currentX < 0) {
                        double DisToGoalX = -currentX + 72;

                    }
                    if (currentX > 0) {
                        double DisToGoalX = currentX;

                    }
                    if (currentY < 0) {
                        double DisToGoalY = -currentY + 72;

                    }
                    if (currentY > 0) {
                        double DisToGoalY = currentY;

                    }
                }

//                double currentX = sigma.position.x;
//                double currentY = sigma.position.y;
                double currentHeading = sigma.heading.log();
                double pinpoint_error = Math.atan2(currentX,currentY);


                //
//                telemetry.addData("Pose", drive.localizer.getPose());
                telemetry.addData("atan2",pinpoint_error);
                telemetry.addData("X",currentX);
                telemetry.addData("Y",currentY);
                telemetry.addData("Head",currentHeading);
                telemetry.update();
            }
        }
    }
}


