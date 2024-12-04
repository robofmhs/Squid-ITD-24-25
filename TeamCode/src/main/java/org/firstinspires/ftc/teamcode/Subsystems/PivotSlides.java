package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class PivotSlides implements Subsystem{
    private DcMotorEx slide;
    private DcMotorEx arm;
    private PIDController slideController;
    public static double slideKp = 0.02, slideKi = 0.00087, slideKd =0.00075;
    public static double slideKf = 0.0008;
    private int slidePos;
    public static int slideTarget;
    private int armPos;
    private double slidePower;
    private boolean isPID =true;
    private double armTicks_in_degree= 5281.1 / 360;
    private double slidePid;

    public PivotSlides(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        arm = map.get(DcMotorEx.class, "arm");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slideController = new PIDController(slideKp,slideKi,slideKd);
    }

    @Override
    public void update() {
        if(isPID) {
            slidePower = getSlideFF() + getSlidePID();
            slide.setPower(slidePower);
        }
        slidePos= Range.clip(slide.getCurrentPosition(),1,2000);
        slideTarget= Range.clip(slideTarget,1,2000);
    }

    public void togglePID(boolean x){
        isPID=x;
    }
    public void setSlideTarget(int x){
        slideTarget=x;
    }
    public int getSlideTarget(){
        return slideTarget;
    }
    public double getSlidePos() {
        return slide.getCurrentPosition();
    }
    public double getSlidePID() {
        slideController.setPID(slideKp, slideKi, slideKd);
        slidePid = slideController.calculate(slidePos, slideTarget);
        return slidePid;
    }
    public double getSlideFF() {
        armPos = arm.getCurrentPosition();
        double slideff = Math.sin(Math.toRadians(armPos/armTicks_in_degree-10))*slideKf;
        return slideff;
    }


}
