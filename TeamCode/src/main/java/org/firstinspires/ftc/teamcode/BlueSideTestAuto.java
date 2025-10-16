package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        //TODO: instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        for (int i = 0; i < 5; i++) {

            Pose2d currentPos;
            if (i == 0) {
                currentPos = initialPose;
            } else {
                currentPos = drive.localizer.getPose();
            }

            TrajectoryActionBuilder tab1 = drive.actionBuilder(currentPos)
                    //.lineToYSplineHeading(24, Math.toRadians(0))
                    .lineToYSplineHeading(24.0, Math.PI / 2)
                    .lineToXSplineHeading(24.0, Math.PI / 2)
                    //.strafeTo(new Vector2d(44.5, 30))
                    .lineToYSplineHeading(-24.0, Math.PI / 2)
                    .lineToXSplineHeading(-24.0, Math.PI / 2);


            Action trajectoryActionChosen = tab1.build();
            Actions.runBlocking(trajectoryActionChosen);

            if (isStopRequested()) {
                return;
            }
        }
    }
}