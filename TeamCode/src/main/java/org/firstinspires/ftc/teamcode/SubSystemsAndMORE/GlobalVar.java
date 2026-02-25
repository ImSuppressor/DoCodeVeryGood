package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import com.acmerobotics.roadrunner.Pose2d;

public class GlobalVar {
    public static double ColorBay1;
    public static double ColorBay2;
    public static double ColorBay3;
    public static String pattern;
    public static double team;
    public static boolean Scanning;
    public static Pose2d LastPose;
    // pipelines LL
    // 1 blue tracking
    // 2 red tracking
    // 0 pattern
    // turret MAX 172.5
    // turret MIN 7.5
    public GlobalVar() {

        pattern = "none";
        ColorBay1 = 0;
        ColorBay2 = 0;
        ColorBay3 = 0;
        team = 1; //2=blue,1=red
        Scanning = true;

        //TODO only put if gives errors

     //   LastPose = new Pose2d(0,0,0);

    }
}
//double powervar = -(.0000000336305)*Math.pow(DistanceToGoal,4)+0.000013082*Math.pow(DistanceToGoal,3)-0.0018165*Math.pow(DistanceToGoal,2)+0.109416*(DistanceToGoal)-1.92276;
//
//double servovar = .0000000919818*Math.pow(DistanceToGoal,4) - 0.0000339204*Math.pow(DistanceToGoal,3)+0.004539*Math.pow(DistanceToGoal,2)-0.25738*DistanceToGoal+5.71927;
//
//            outakeL.setPower(-powervar);
//            outakeR.setPower(powervar);
//            hood.setPosition(servovar);
//            telemetry.addData("Distx",DisToGoalX);
//            telemetry.addData("DistY",DisToGoalY);
//            telemetry.addData("Tx",result.getTx());
//How to Tune $K_v$Tuning Feedforward is different from tuning PID:Set P, I, and D to 0.Have your robot spin in a circle at a constant speed while looking at the AprilTag.Increase $K_v$ until the turret stays roughly pointed at the tag while the robot is spinning. It won't be perfect (it will drift), but it should "shadow" the tag.Then turn your PID back on. The PID now only has to clean up the small remaining error, rather than doing all the heavy lifting. This makes the tracking feel "locked in" even at high speeds.Why this is better for AprilTags:AprilTag pipelines often have slightly more latency (delay) than simple color tracking. If you rely only on PID, the turret will always be "behind" the tag by a few degrees. $K_v$ eliminates that lag by giving the motor the "push" it needs the moment it detects the target is moving.Would you like me to show you how to pull the robot's actual rotation speed from your MecanumDrive so the turret can compensate for your driving even more accurately?