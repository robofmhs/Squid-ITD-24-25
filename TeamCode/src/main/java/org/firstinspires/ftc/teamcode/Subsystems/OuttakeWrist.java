package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class OuttakeWrist implements Subsystem {
    private Servo wrist;
    public static double wristPos;
    public OuttakeWrist(HardwareMap map) {
        wrist = map.get(Servo.class, "wrist");
        wrist.setPosition(0);
    }
    @Override
    public void update() {
        wrist.setPosition(wristPos);
        wristPos= Range.clip(wrist.getPosition(),0,.81);
    }
    public void setWristPos(double pos) {
        wristPos = pos;
    }
    public void turnWrist(double x) {
        wristPos += x;
    }
    public double getWristPos(){
        return wristPos;
    }
}
