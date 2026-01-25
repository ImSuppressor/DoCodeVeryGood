package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;

import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;


import android.graphics.Color;

import androidx.annotation.NonNull;



import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EpsilonOpModes.BlueDepot;


public class ShootNow {

    private final HardwareMap hwMap;

    public ShootNow(HardwareMap hardwareMap) {



        this.hwMap = hardwareMap;

    }

    public class Shoot implements Action {

        private final Servo Bay1Boot;

        private final Servo Bay2Boot;

        private final Servo Bay3Boot;

        private final ElapsedTime time2 = new ElapsedTime();

        private boolean initialized = false;
        private boolean shooting1 = false;
        private boolean shooting2 = false;
        private boolean shooting3 = false;
        public double shoot = .5;

        public double ready = 0.95;

        public double cycle = 1;
        public double cycleDown = .75;

        public Shoot() {

// Initialize hardware ONCE here

            Bay1Boot = hwMap.get(Servo.class, "Boot1");

            Bay2Boot = hwMap.get(Servo.class, "Boot2");

            Bay3Boot = hwMap.get(Servo.class, "Boot3");


        }

        @Override

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!initialized) {
                Bay1Boot.setPosition(ready);
                Bay2Boot.setPosition(ready);
                Bay3Boot.setPosition(ready);
                time2.reset();
                initialized = true;

            }
            double time = time2.seconds();

                if (pattern.equals("PPG")) {

                    if (time < (cycle) && !shooting1) {

                        if (ColorBay1 == 1) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting1 = true;

                        } else if (ColorBay2 == 1) {


                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting1 = true;

                        } else if (ColorBay3 == 1) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting1 = true;

                        }
                    } else if (time > (cycle-cycleDown) && time < cycle) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                    } else if (time < 2 * cycle && time > cycle && !shooting2) {

                        if (ColorBay1 == 1) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting2 = true;

                        } else if (ColorBay2 == 1) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting2 = true;

                        } else if (ColorBay3 == 1) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting2 = true;

                        }

                    }  else if (time > ((2*cycle)-cycleDown) && time < 2*cycle) {
                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                    } else if (time < 3 * cycle && time > 2 * cycle && !shooting3) {

                        if (ColorBay1 == 2) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting3 = true;

                        } else if (ColorBay2 == 2) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting3 = true;

                        } else if (ColorBay3 == 2) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting3 = true;

                        }
                    }
                }
                else if (pattern.equals("PGP")) {

                    if (time < cycle && !shooting1) {

                        if (ColorBay1 == 1) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting1 = true;

                        } else if (ColorBay2 == 1) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting1 = true;

                        } else if (ColorBay3 == 1) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting1 = true;

                        }
                     } else if (time > (cycle-cycleDown) && time < cycle) {

                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                    }

                    else if (time < 2 * cycle & time > cycle && !shooting2) {

                        if (ColorBay1 == 2) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting2 = true;

                        } else if (ColorBay2 == 2) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting2 = true;

                        } else if (ColorBay3 == 2) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting2 = true;

                        }
                    } else if (time > (2*cycle-cycleDown) && time < 2*cycle) {

                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                    }

                    else if (time < 3 * cycle & time > 2 * cycle && !shooting3) {

                        if (ColorBay1 == 1) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting3 = true;

                        } else if (ColorBay2 == 1) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting3 = true;

                        } else if (ColorBay3 == 1) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting3 = true;

                        }
                    }
                }

                else if (pattern.equals("GPP")) {

                    if (time < cycle && !shooting1) {

                        if (ColorBay1 == 2) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting1 = true;

                        } else if (ColorBay2 == 2) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting1 = true;

                        } else if (ColorBay3 == 2) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting1 = true;

                        }
                    } else if (time > (cycle-cycleDown) && time < cycle) {

                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                    }

                    else if (time < 2 * cycle && time > cycle && !shooting2) {

                        if (ColorBay1 == 1) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting2 = true;

                        } else if (ColorBay2 == 1) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting2 = true;

                        } else if (ColorBay3 == 1) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting2 = true;

                        }
                    } else if (time > (2 * cycle-cycleDown) && time < 2*cycle) {

                        Bay1Boot.setPosition(ready);
                        Bay2Boot.setPosition(ready);
                        Bay3Boot.setPosition(ready);

                    }

                    else if (time < 3 * cycle && time > 2 * cycle && !shooting3) {

                        if (ColorBay1 == 1) {

                            Bay1Boot.setPosition(shoot);

                            ColorBay1 = 0;
                            shooting3 = true;

                        } else if (ColorBay2 == 1) {

                            Bay2Boot.setPosition(shoot);

                            ColorBay2 = 0;
                            shooting3 = true;

                        } else if (ColorBay3 == 1) {

                            Bay3Boot.setPosition(shoot);

                            ColorBay3 = 0;
                            shooting3 = true;

                        }
                    }
                }

            return time < 3;
        }
    }
    public Action shoot() {
        return new Shoot();
    }
}