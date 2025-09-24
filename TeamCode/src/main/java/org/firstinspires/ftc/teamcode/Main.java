package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Main TeleOp V0.00-3.34", group="Linear OpMode")

public class Main extends LinearOpMode {

    public List<MotorFeature> featureList = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        featureList = new ArrayList<>();
        featureList.add(new MotorDrive());
        featureList.add(new LimeLight());
        for (MotorFeature feature: featureList){
            feature.init(hardwareMap);
        }

       // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           for (MotorFeature feature: featureList){
                List<String> messageList = feature.driveLoop(gamepad1, gamepad2);
                for (String message : messageList) {
                    telemetry.addLine(message);
                }
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            if (gamepad2.start) {
                stop(featureList);
            }

        }
    }

    protected void stop(List<MotorFeature>  featureList){
        for (MotorFeature feature: featureList) {
            feature.stop();
        }
    }
    protected void goToPosition(List<MotorFeature>  featureList, Position position){
        for (MotorFeature feature: featureList) {
            feature.goToPosition(position);
        }
    }


}

