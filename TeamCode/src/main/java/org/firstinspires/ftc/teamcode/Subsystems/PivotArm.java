package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class PivotArm implements Subsystem {
    private DcMotorEx arm;
    private DcMotorEx arm2;
    private DcMotorEx slide;
    private double armPos;
    private PIDController armController;
    public static double armKp = 0.0023, armKi = 0.000, armKd = 0.00006;
    private double armIncrement = 10;
    public static double armKf = 0.0001;
    public static int armTarget = 0;
    private final double armTicks_in_degree = 5281.1 / 360;
    private double slidePos;
    private double armPower;
    private double armAngle;
    private boolean isPID=true;
    private InterpLUT armKfLUT = new InterpLUT();
    private double armPid;

    public PivotArm(HardwareMap map) {
        arm = map.get(DcMotorEx.class, "arm");
        arm2 = map.get(DcMotorEx.class, "arm2");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide = map.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armController = new PIDController(armKp, armKi, armKd);
        armKfLUT.add(0, 0.001);
        armKfLUT.add(500, .035);
        armKfLUT.add(1000, .07);
        armKfLUT.add(1500, .095);
        armKfLUT.add(2000, 0.12);
        armKfLUT.add(2100, 0.163);
        armKfLUT.createLUT();

    }

    @Override
    public void update() {
        if(isPID) {
            double pid2 = Math.signum(getArmPID())*Math.sqrt(Math.abs(getArmPID()));
            armPower = getArmFF() + pid2;
            arm.setPower(armPower);
            arm2.setPower(armPower);
        }
        armPos=Range.clip(arm.getCurrentPosition(),-100,2200);
        armTarget=Range.clip(armTarget,-100,2200);
    }
    public void togglePID(boolean x){
        isPID=x;
    }
    public void setArmTarget(int x){
        armTarget=x;
    }
    public int getArmTarget(){
        return armTarget;
    }
    public int getArmPos(){
        return arm.getCurrentPosition();
    }
    public double getArmPID(){
        armController.setPID(armKp, armKi, armKd);
        armPid = armController.calculate(armPos, armTarget);
        return armPid;
    }
    public double getArmFF() {
        slidePos = Range.clip(slide.getCurrentPosition(),1,2000);
        armKf= armKfLUT.get(slidePos);
        armAngle= armPos/armTicks_in_degree-10;
        double armff = Math.cos(Math.toRadians(armAngle))*armKf;
        return armff;
    }
}
