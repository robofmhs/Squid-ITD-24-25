package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Base {
//    MOTOR_RPM(435),
//    GEAR_RATIO(1),
//    WHEEL_DIAMETER_MM(96),
//    LENGTH(16),
//    WIDTH(16),;

    public int MOTOR_RPM=435;
    public double GEAR_RATIO=1.0;
    public double WHEEL_DIAMETER_MM=96.0;
    public double LENGTH = 16.0;
    public double WIDTH = 16.0;

    public DcMotorEx FRONTLEFT= hardwareMap.get(DcMotorEx.class,"fl");
    public DcMotorEx FRONTRIGHT= hardwareMap.get(DcMotorEx.class,"fr");
    public DcMotorEx BACKLEFT= hardwareMap.get(DcMotorEx.class,"bl");
    public DcMotorEx BACKRIGHT= hardwareMap.get(DcMotorEx.class,"br");


}
