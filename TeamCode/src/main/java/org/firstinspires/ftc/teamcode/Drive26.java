package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Drive26")
public class Drive26 extends LinearOpMode {

    private Servo pin;

    /**S
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        pin = hardwareMap.get(Servo.class, "pin");
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            telemetry.addData("patty",GlobalVar.pattern);

            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.a) {
                    pin.setPosition(0.1);
                } else {
                    pin.setPosition(0.95);
                }
                telemetry.update();
            }
        }
    }
}


