package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

public class GlobalVar {
    public static double ColorBay1;
    public static double ColorBay2;
    public static double ColorBay3;
    public static String pattern;
    public static double team;
    public static boolean Scanning;

    public GlobalVar() {
        pattern = "none";
        ColorBay1 = 0;
        ColorBay2 = 0;
        ColorBay3 = 0;
        team = 1; //1 means red, 2 means blue
        Scanning = true;
    }
}
