package org.firstinspires.ftc.teamcode.Controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeGripper;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeWrist;
import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.PivotSlides;

public class OuttakeControl {
    public OuttakeWrist wrist;
    public OuttakeGripper gripper;
    public PivotArm arm;
    public PivotSlides slides;

    public OuttakeControl(OuttakeWrist wrist, OuttakeGripper gripper, PivotArm arm, PivotSlides slides) {
        this.wrist = wrist;
        this.gripper = gripper;
        this.arm = arm;
        this.slides = slides;
    }
    public void update(Gamepad g1, Gamepad g2) {
        if(g2.left_stick_y>0){
            wrist.turnWrist(0.05);
        }
        else if(g2.left_stick_y<0){
            wrist.turnWrist(-0.05);
        }

        arm.setArmTarget((int)(g2.right_stick_y*20)+arm.getArmTarget());
        double slideTrig = g2.right_trigger-g2.left_trigger;
        slides.setSlideTarget((int)(1.0*slideTrig*100)+ slides.getSlideTarget());

        if(g2.a){
            gripper.toggleGripper();
        }


        wrist.update();
        gripper.update();
        arm.update();
        slides.update();
    }
}
