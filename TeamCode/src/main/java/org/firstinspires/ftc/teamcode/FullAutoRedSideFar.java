package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "FULL_AUTO_RED_FAR_PIXEL", group = "Autonomous")
public class FullAutoRedSideFar extends AbstractFullAuto {


    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(63, 15, Math.toRadians(0));

    }

    @Override
    public void setOuttakePower() {
        outtakemotorright.setPower(-0.46);
        outtakemotorleft.setPower(0.46);
    }

    @Override
    public void runFirstPath(MecanumDrive drive, Pose2d initialPose) {
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(53, 15), Math.toRadians(180))
                .turn(Math.toRadians(-22));
        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);


    }

    @Override
    public void runSecondPath(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
                .strafeToConstantHeading(new Vector2d(36, 20))
                .turn(Math.toRadians(110));
        trajectoryActionChosen = goToIntake.build();
        Actions.runBlocking(trajectoryActionChosen);

        intakemotor.setPower(0.75);

        TrajectoryActionBuilder adjustIntakePos = drive.actionBuilder(getCurrentPos(drive))
                .splineToConstantHeading(new Vector2d(36, 45), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(36, 15), Math.toRadians(0));
        trajectoryActionChosen = adjustIntakePos.build();
        Actions.runBlocking(trajectoryActionChosen);
        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
                .turn(Math.toRadians(-110))

                .splineToConstantHeading(new Vector2d(53, 15), Math.toRadians(90));

        trajectoryActionChosen = goToLaunchSpot2.build();
        Actions.runBlocking(trajectoryActionChosen);
    }

    @Override
    public void parkOutsideLaunch(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToPark = drive.actionBuilder(getCurrentPos(drive))
                .splineToConstantHeading(new Vector2d(36, 45), Math.toRadians(-90));
        trajectoryActionChosen = goToPark.build();
        Actions.runBlocking(trajectoryActionChosen);

    }

}