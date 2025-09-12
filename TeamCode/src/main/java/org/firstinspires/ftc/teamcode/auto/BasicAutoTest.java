package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.setup.MecanumDrive;

@Config
@Autonomous(name = "Basic Auto Test", group = "Autonomous")
public class BasicAutoTest extends LinearOpMode {
    public Servo Light = null;
    @Override
    public void runOpMode() {
        //Define any stored poses (X/Y coordinates with heading)
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));

        //Define any stored vectors (X/Y coordinate)

        //Initialize systems

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Light = hardwareMap.get(Servo.class, "Light");

        //Define trajectories
        //Trajectories must be given a start pose (not vector)
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(3)
                .strafeTo(new Vector2d(12, 0))
                .waitSeconds(3)
                .strafeTo(new Vector2d(12, 12))
                .waitSeconds(3)
                .strafeTo(new Vector2d(0, 0))
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .waitSeconds(3)
                .strafeTo(new Vector2d(12, 0))
                .waitSeconds(3)
                .strafeTo(new Vector2d(12, 12))
                .waitSeconds(3)
                .strafeTo(new Vector2d(0, 0))
                .waitSeconds(3);


        //Close out trajectory
        //TODO: What does this actually do?
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.

        //Wait for start
        waitForStart();

        if (isStopRequested()) return;

        //Builds out trajectories
        Action tab1Built = tab1.build();
        Action tab2Built = tab2.build();
        //Runs a sequence of actions.
        Actions.runBlocking(
                new SequentialAction(
                        tab1Built,
                        tab2Built,
                        trajectoryActionCloseOut
                )
        );
    }
}