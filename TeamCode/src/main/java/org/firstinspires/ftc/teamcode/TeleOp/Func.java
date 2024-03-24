package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.TeleOp.RC_Base;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Func extends LinearOpMode {
    RC_Base thing = new RC_Base();
    void closeGripper() {
        if (thing.guard.getPosition() >= 0.7 && thing.guard.getPosition() < 0.71) {
            thing.guard.setPosition(.2);

        } else if (thing.guard.getPosition() >= .2) {
            thing.guard.setPosition(0.7);

        } else {
            thing.guard.setPosition(0.2);
        }

    }
    void setPOS1(){
            thing.guard.setPosition(.6);
            thing.arm.setVelocity(4000);
            thing.arm.setTargetPosition(-2614);
            thing.arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sleep(1500);
            thing.wrist.setPosition(.79);}
    void setPos2(){

            thing.arm.setVelocity(3000);
            thing.arm.setVelocity(4000);
            thing.arm.setTargetPosition(-1906);
            thing.arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            sleep(500);
            thing.arm.setTargetPosition(0);
            thing.wrist.setPosition(.46);
            sleep(500);
            thing.arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


    }
    void oneDrop(){
        thing.guard.setPosition(.2);
        sleep(300);
        thing.guard.setPosition(.7);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
