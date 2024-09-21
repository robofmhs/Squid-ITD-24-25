package org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {
//    ARM_RPM(435),
//    GRIPPER_OPEN(0.5),
//    GRIPPER_CLOSE(0.0);
    public int ARM_RPM = 435;
    public double GRIPPER_LEFT_OPEN = 0.5;
    public double GRIPPER_LEFT_CLOSE = 0.0;
    public double GRIPPER_RIGHT_OPEN = 0.5;
    public double GRIPPER_RIGHT_CLOSE = 0.0;
    public double WRIST_INCREMENTS = 0.01;


    public DcMotorEx ARM= hardwareMap.get(DcMotorEx.class,"arm"); // is port 0 on expansion
    public Servo GRIPPER_LEFT= hardwareMap.get(Servo.class,"gl"); // is port 2 in expansion
    public Servo GRIPPER_RIGHT= hardwareMap.get(Servo.class,"gr"); // is port 1 in expansion
    public Servo WRIST= hardwareMap.get(Servo.class,"wrist"); // is port 0 in expansion


    public void gripper(Servo servo1, double pos1,double pos2){
        if (servo1.getPosition() >= pos1 && servo1.getPosition() < pos1+.01) {
            servo1.setPosition(pos2);

        } else if (servo1.getPosition() >= pos2) {
            servo1.setPosition(pos1);

        } else {
            servo1.setPosition(pos2);
        }
    }
    public void gripper2(Servo servo1, double pos1,double pos2,Servo servo2, double pos12,double pos22){
        this.gripper(servo1,pos1,pos2);
        this.gripper(servo2,pos12,pos22);
    }

}
