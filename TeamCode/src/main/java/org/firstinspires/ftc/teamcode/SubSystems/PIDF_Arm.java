package org.firstinspires.ftc.teamcode.SubSystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    InterpLUT lut = new InterpLUT();
    private ElapsedTime runtime = new ElapsedTime();

    //arm stuff
    private PIDController armController;

    public static double armKp = 0.00, armKi = 0, armKd =0.0000;
    public static double armKf = 0.0001;
    public static int armTarget = 0;
    public DcMotorEx arm;
    private final double armTicks_in_degree = 5281.1/360;
    private double armAngle;

    //slide stuff
    private PIDController slideController;

    public static double slideKp = 0.00, slideKi = 0, slideKd =0.0000;
    public static double slideKf = 0.0001;
    public static int slideTarget = 0;
    public DcMotorEx slide;
    private final double slideTicks_in_degree = 537.7/360;

    @Override
    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController = new PIDController(armKp, armKi, armKd);
        slideController = new PIDController(slideKp, slideKi, slideKd);
        //lut has slide ticks as input, has output as armKf
        lut.add(0, 0);
        lut.createLUT();

    }

    @Override
    public void loop() {
        armController.setPID(armKp, armKi, armKd);
        int armPos = arm.getCurrentPosition();
        double armPid = armController.calculate(armPos, armTarget);
        armAngle= armTarget/armTicks_in_degree-85;
        double armff = Math.cos(Math.toRadians(armAngle)* armKf);
//        double ff = Math.sin(Math.toRadians(armPos/armTicks_in_degree+5)*armKf);
        double armPower = armPid + armff;
        arm.setPower(armPower);

        slide.setTargetPosition(0);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        slideController.setPID(slideKp, slideKi, slideKd);
//        int slidePos = slide.getCurrentPosition();
//        double slidePid = slideController.calculate(slidePos, slideTarget);
////        armAngle= armTarget/armTicks_in_degree-85;
////      double ff = Math.cos(Math.toRadians(armAngle)* armKf);
//        double slideff = Math.sin(armAngle*slideKf);
//        double slidePower = slidePid + slideff;
//        arm.setPower(slidePower);
    }
}
