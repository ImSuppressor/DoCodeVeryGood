package subsystems;

//import static subsystems.Spindexer.spindexerState.OFF;
//import static subsystems.Spindexer.spindexerState.ON;

import android.transition.ChangeBounds;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;


@Config
public class Spindexer implements Component {


    private int targetPosition;
    public static double TICKS_PER_REVOLUTION = 288;

    public static double GEAR_RATIO = 1.0;

    public static final double TICKS_PER_DEGREE = (TICKS_PER_REVOLUTION * GEAR_RATIO) / 360.0;


    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx spindexerMotor;
    public SpindexerState spindexerState;
    public enum SpindexerState {
        OFF,
        ON
    }

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;

        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");

        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.spindexerState = SpindexerState.OFF;

        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void rotateDegrees(double degrees){
        if (spindexerState == SpindexerState.ON){
            return;
        }

        targetPosition = spindexerMotor.getCurrentPosition() + (int)(degrees * TICKS_PER_DEGREE);

        spindexerMotor.setTargetPosition(targetPosition);

        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexerMotor.setPower(0.2);
        spindexerState = SpindexerState.ON;
    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (spindexerState) {
            case OFF:
                spindexerMotor.setPower(0);
                break;
            case ON:
                spindexerMotor.setPower(0.1);
                break;
        }
    }

    @Override
    public String test() {
        return null;
    }


}
