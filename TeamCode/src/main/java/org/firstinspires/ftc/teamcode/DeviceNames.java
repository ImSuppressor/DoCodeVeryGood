package org.firstinspires.ftc.teamcode;

public enum DeviceNames {
    LF_MOTOR("left_front_drive"),
    LB_MOTOR("left_back_drive"),
    RF_MOTOR("right_front_drive"),
    RB_MOTOR("right_back_drive"),
    ARM("arm"),
    SEC_ARM("secondArm"),
    SLIDE("slide"),
    INTAKE("pinch"),
    WRIST1("wrist"),
    WRIST2("wrist2"),
    WRIST3("wrist3"),
    IMU("imu"),
    ODOMCONTROLLER("wheel");





    private final String name;


    private DeviceNames(String name) {
        this.name = name;

    }


    public String toString() {
        return name;
    }

}
