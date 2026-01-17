package org.firstinspires.ftc.teamcode.EpsilonOpModes;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.team;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RandomBSfromRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar;

@TeleOp(name = "Drive26")
public class Drive26 extends LinearOpMode {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    /**
     * Describe this function...
     */
    private void Sticks(double speed) {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        fl.setPower(speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        fr.setPower(-speed * gamepad1.right_stick_x + (-(speed * gamepad1.left_stick_x) - speed * gamepad1.left_stick_y));
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        bl.setPower(speed * gamepad1.right_stick_x + (-speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        br.setPower(-speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
    }



//    private Servo pin;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, -68, 45));
        double red_goalX = 72;////red is at 72, 72
        double red_goalY = 72;
        double blue_goalX = 0;
        double blue_goalY = -72;
        double speed;



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
                double currentY = getCurrentY(sigma, currentX);

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

    private static double getCurrentY(Pose2d sigma, double currentX) {
        double currentY = sigma.position.y;
        if (team==1) {
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
        if (team==2) {
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
        return currentY;
    }
}


