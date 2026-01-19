//package org.firstinspires.ftc.teamcode.SubSystemsAndMORE;
//
//import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay1;
//import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay2;
//import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.ColorBay3;
//import static org.firstinspires.ftc.teamcode.SubSystemsAndMORE.GlobalVar.pattern;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//public class colorSense {
//    public static class ColorSense implements Action {
//        private final NormalizedColorSensor bay11, bay12, bay21, bay22, bay31, bay32;
//        private final DistanceSensor dist11, dist12, dist21, dist22, dist31, dist32;
//        private final Limelight3A Limelight;
//        private boolean initialized = false;
//        private final ElapsedTime timer;
//
//
//        public ColorSense(HardwareMap hwMap) {
//            this.bay11 = hwMap.get(NormalizedColorSensor.class, "Bay1.1");
//            this.bay12 = hwMap.get(NormalizedColorSensor.class, "Bay1.2");
//            this.dist11 = hwMap.get(DistanceSensor.class, "Bay1.1");
//            this.dist12 = hwMap.get(DistanceSensor.class, "Bay1.2");
//            this.bay21 = hwMap.get(NormalizedColorSensor.class, "Bay2.1");
//            this.bay22 = hwMap.get(NormalizedColorSensor.class, "Bay2.2");
//            this.dist21 = hwMap.get(DistanceSensor.class, "Bay2.1");
//            this.dist22 = hwMap.get(DistanceSensor.class, "Bay2.2");
//            this.bay31 = hwMap.get(NormalizedColorSensor.class, "Bay3.1");
//            this.bay32 = hwMap.get(NormalizedColorSensor.class, "Bay3.2");
//            this.dist31 = hwMap.get(DistanceSensor.class, "Bay3.1");
//            this.dist32 = hwMap.get(DistanceSensor.class, "Bay3.2");
//            this.Limelight = hwMap.get(Limelight3A.class, "limelight");
//            this.timer = new ElapsedTime();
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (!initialized) {
//                ColorBay1 = 0;
//                ColorBay2 = 0;
//                ColorBay3 = 0;
//                timer.reset();
//                initialized = true; // Prevents this block from running again
//            }
////            LLResult result = Limelight.getLatestResult();
////            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
////                if (pattern.equals("none")) {
////                    if (fiducial.getFiducialId() == 21) {
////                        pattern = "GPP";
////                    } else if (fiducial.getFiducialId() == 22) {
////                        pattern = "PGP";
////                    } else if (fiducial.getFiducialId() == 23) {
////                        pattern = "PPG";
////                    }
////                }
////            }
//
//            NormalizedRGBA colors11 = bay11.getNormalizedColors();
//            NormalizedRGBA colors12 = bay12.getNormalizedColors();
//            NormalizedRGBA colors21 = bay21.getNormalizedColors();
//            NormalizedRGBA colors22 = bay22.getNormalizedColors();
//            NormalizedRGBA colors31 = bay31.getNormalizedColors();
//            NormalizedRGBA colors32 = bay32.getNormalizedColors();
//
//            double avgBlue1 = (colors11.blue + colors12.blue) / 2.0;
//            double avgGreen1 = (colors11.green + colors12.green) / 2.0;
//
//            double avgBlue2 = (colors21.blue + colors22.blue) / 2.0;
//            double avgGreen2 = (colors21.green + colors22.green) / 2.0;
//
//            double avgBlue3 = (colors31.blue + colors32.blue) / 2.0;
//            double avgGreen3 = (colors31.green + colors32.green) / 2.0;
//
//            double distance1 = Math.min(dist11.getDistance(DistanceUnit.CM), dist12.getDistance(DistanceUnit.CM));
//            double distance2 = Math.min(dist21.getDistance(DistanceUnit.CM), dist22.getDistance(DistanceUnit.CM));
//            double distance3 = Math.min(dist31.getDistance(DistanceUnit.CM), dist32.getDistance(DistanceUnit.CM));
//
//            if (distance1 < 3) {
//                if (avgBlue1 > avgGreen1) {
//                    ColorBay1 = 1;
//                } else if (avgGreen1 > avgBlue1){
//                    ColorBay1 = 2;
//                } else {
//                    ColorBay1 = 0;
//                }
//            }
//            if (distance2 < 3) {
//                if (avgBlue2 > avgGreen2) {
//                    ColorBay2 = 1;
//                } else if (avgGreen2 > avgBlue2){
//                    ColorBay2 = 2;
//                } else {
//                    ColorBay2 = 0;
//                }
//            }
//            if (distance3 < 10) {
//                if (avgBlue3 > avgGreen3) {
//                    ColorBay3 = 1;
//                } else if (avgGreen3 > avgBlue3){
//                    ColorBay3 = 2;
//                } else {
//                    ColorBay3 = 0;
//                }
//            }
//
//            return timer.seconds() < .4;
//        }
//
//
//    }
//}
