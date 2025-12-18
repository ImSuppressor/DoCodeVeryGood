package org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
public class Robot {
    private static final Flywheel flywheel = new Flywheel(hardwareMap.dcMotor.get("flywheelMotor"));

    public Robot(){
    }

    public Flywheel getFlywheel(){
        return this.flywheel;
    }
}
