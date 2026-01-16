package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;
import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootNow {

    double shoot;
    double ready;
    double cycle;


    public ShootNow(HardwareMap hardwareMap) {
        this.shoot = .6;
        this.ready = .95;
        this.cycle = 1;
    }

    public class Shoot implements Action {
        Servo Bay1Boot = hardwareMap.get(Servo.class, "Boot1");
        Servo Bay2Boot = hardwareMap.get(Servo.class, "Boot2");
        Servo Bay3Boot = hardwareMap.get(Servo.class, "Boot3");
        ElapsedTime time2 = new ElapsedTime();



        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (pattern.equals("PPG")) {
                if (time2.seconds() < cycle) {
                    telemetry.addLine("ball1");
                    if (ColorBay1.equals("Purple1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (time2.seconds() < 2 * cycle & time2.seconds() > cycle) {
                    telemetry.addLine("ball2");

                    if (ColorBay1.equals("Purple2")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (time2.seconds() < 3 * cycle & time2.seconds() > 2 * cycle) {
                    telemetry.addLine("ball3");
                    if (ColorBay1.equals("Green1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Green1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Green1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
            }
            if (pattern.equals("PGP")) {
                if (time2.seconds() < cycle) {

                    if (ColorBay1.equals("Purple1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (time2.seconds() < 2 * cycle & time2.seconds() > cycle) {

                    if (ColorBay1.equals("Green1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Green1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Green1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (time2.seconds() < 3 * cycle & time2.seconds() > 2 * cycle) {

                    if (ColorBay1.equals("Purple2")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
            }


            if (pattern.equals("GPP")) {
                if (time2.seconds() < cycle) {
                    if (ColorBay1.equals("Green1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Green1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Green1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (time2.seconds() < 2 * cycle & time2.seconds() > cycle) {

                    if (ColorBay1.equals("Purple1")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple1")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
                if (time2.seconds() < 3 * cycle & time2.seconds() > 2 * cycle) {
                    if (ColorBay1.equals("Purple2")) {

                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                        Bay1Boot.setPosition(shoot);
                    } else if (ColorBay2.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);

                        Bay3Boot.setPosition(ready);

                        Bay2Boot.setPosition(shoot);
                    } else if (ColorBay3.equals("Purple2")) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);


                        Bay3Boot.setPosition(shoot);
                    }
                }
            }

            if (time2.seconds() > 3 * cycle) {
                Bay1Boot.setPosition(ready);
                Bay2Boot.setPosition(ready);
                Bay3Boot.setPosition(ready);
            }
            telemetry.addData("Pattern", pattern);
//            telemetry.addData("time", time2.seconds());
            telemetry.update();
            return time2.seconds() > 3;

        }
    }

    public Action shoot() {
        return new Shoot();

    }
}
