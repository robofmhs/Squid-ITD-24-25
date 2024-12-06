package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class IntakeSlides implements Subsystem{
    private Servo IntakeSlide1;
//    private Servo IntakeSlide2;
    public static double slidePos;
    public IntakeSlides(HardwareMap map){
        IntakeSlide1 =map.get(Servo.class,"IntakeSlide1");
        IntakeSlide1.setPosition(.445);
//        IntakeSlide2 =map.get(Servo.class,"IntakeSlide2");
//        IntakeSlide2.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void update(){
        IntakeSlide1.setPosition(slidePos);
//        IntakeSlide2.setPosition(slidePos);
        slidePos= Range.clip(IntakeSlide1.getPosition(),.445,.77);

    }
    public double getPosition(){return IntakeSlide1.getPosition();}
    public void setPosition(double pos){
        slidePos= Range.clip(pos,.445,.77);

    }
    public void changePosition(double x){
        slidePos= Range.clip(slidePos+x,.445,.77);
    }
}
