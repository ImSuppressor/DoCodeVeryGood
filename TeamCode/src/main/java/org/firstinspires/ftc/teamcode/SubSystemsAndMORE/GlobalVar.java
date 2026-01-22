package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

public class GlobalVar {
    public static double ColorBay1;
    public static double ColorBay2;
    public static double ColorBay3;
    public static String pattern;
    public static double team;
    public static boolean Scanning;
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



    }
}
