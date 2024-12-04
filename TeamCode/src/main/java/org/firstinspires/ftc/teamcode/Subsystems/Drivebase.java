package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivebase implements Subsystem {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;

    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;

    public Drivebase(HardwareMap map) {
        flMotor = map.get(DcMotor.class, "fl");
        frMotor = map.get(DcMotor.class, "fr");
        blMotor = map.get(DcMotor.class, "bl");
        brMotor = map.get(DcMotor.class, "br");
        frMotor.setDirection(DcMotorEx.Direction.REVERSE);
        brMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
    @Override
    public void update(){
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    public void setPower(double y,double x,double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         flPower = (y + x + rx) / denominator;
         blPower = (y - x + rx) / denominator;
         frPower = (y - x - rx) / denominator;
         brPower = (y + x - rx) / denominator;
    }
}
