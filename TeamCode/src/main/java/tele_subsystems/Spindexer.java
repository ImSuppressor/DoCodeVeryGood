package tele_subsystems;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
public class Spindexer implements Component {


//    private LinearOpMode opmode;
//
//    public Spindexer(LinearOpMode opmode, HardwareMap hardwareMap) {
//        this.opmode = opmode;
//        spindexerMotor = hardwareMap.get(DcMotor.class, "spindexer");
//    }

    public static double SPINDEXER_TICKS_PER_REVOLUTION = 288;

    public static double SPINDEXER_GEAR_RATIO = 1.0;

    public static final double SPINDEXER_TICKS_PER_DEGREE = (SPINDEXER_TICKS_PER_REVOLUTION * SPINDEXER_GEAR_RATIO) / 360.0;


    private HardwareMap map;
    private Telemetry telemetry;
    public SpindexerState spindexerState;
    private int spindexerTargetPosition;
    public enum SpindexerState {
        OFF,
        ON,
        NORMAL
    }
    public DcMotorEx spindexerMotor;
    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.map = hardwareMap;
        this.telemetry = telemetry;


        spindexerMotor = map.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        spindexerState = SpindexerState.NORMAL;

    }

    public void rotateDegrees(double degrees){

            telemetry.addData("in rotateDegrees", spindexerMotor.isBusy());
            telemetry.update();
            spindexerTargetPosition = spindexerMotor.getCurrentPosition() + (int)(degrees * SPINDEXER_TICKS_PER_DEGREE);
            spindexerMotor.setTargetPosition(spindexerTargetPosition);
            spindexerMotor.setTargetPositionTolerance(2);
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexerMotor.setPower(0.3);

            spindexerState = SpindexerState.ON;



    }
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }
//    public void rotate120degrees(){
//        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        spindexerMotor.setTargetPosition(96);
//        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spindexerMotor.setPower(0.5);
//        if (spindexerMotor.getCurrentPosition() == 96){
//            spindexerState = spindexerState.OFF;
//        }
//    }


    @Override
    public void reset() {

    }

    @Override
    public void update() {
        switch (spindexerState) {
            case OFF:
                if (spindexerMotor.getPower() != 0) {
                    spindexerMotor.setPower(0);
                }
                break;
            case ON:
                if (!spindexerMotor.isBusy()) {
                    spindexerMotor.setPower(0);
                    spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexerState = SpindexerState.NORMAL;
                }
                break;
            case NORMAL:
                break;
        }
    }

    public boolean isSpindexerBusy(){
        return spindexerMotor.isBusy();
    }


@Override
public String test() {
    return null;
}


}

