package org.firstinspires.ftc.teamcode.SubSystemsNot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class PIDF_Arm extends OpMode {
    InterpLUT lut = new InterpLUT();
    private ElapsedTime runtime = new ElapsedTime();

    //arm stuff
    private PIDController armController;

    public static double armKp = 0.0, armKi = 0.000, armKd =0.000;
    public static double armKf = 0.0001;
    public static int armTarget = 0;
    public DcMotorEx arm;
    private final double armTicks_in_degree = 5281.1/360;
    private double armAngle;

    //slide stuff
    private PIDController slideController;

    public static double slideKp = 0.02, slideKi = 0.00087, slideKd =0.00075;
    public static double slideKf = 0.0008;
    public static int slideTarget = 0;
    public DcMotorEx slide;
    private final double slideTicks_in_degree = 537.7/360;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        armController = new PIDController(armKp, armKi, armKd);
        slideController = new PIDController(slideKp, slideKi, slideKd);
        //lut has slide ticks as input, has output as armKf
        lut.add(0, 0.038);
        lut.add(500, .093);
        lut.add(1000, .13);
        lut.add(1500, .14);
        lut.add(2000, 0.163);
        lut.add(2100, 0.163);
        lut.createLUT();

    }

    @Override
    public void loop() {
        int slidePos = Range.clip(slide.getCurrentPosition(),1,2000);
        slideTarget= Range.clip(slideTarget,1,2000);
        armController.setPID(armKp, armKi, armKd);
        int armPos = arm.getCurrentPosition();
        double armPid = armController.calculate(armPos, armTarget);
        armAngle= armPos/armTicks_in_degree-10;
        double rijArmgle = Math.abs(armAngle)-90;
        double Paraslide = (2.0/3.0)-(rijArmgle/270);
        armKf=lut.get(slidePos);
        double armff = Math.cos(Math.toRadians(armAngle))*armKf;
//        double ff = Math.sin(Math.toRadians(armPos/armTicks_in_degree+5)*armKf);
        double armPower = armPid + armff;
        arm.setPower(armPower);

//        slide.setPower(.5);
//        slide.setTargetPosition(slideTarget);
//        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideController.setPID(slideKp, slideKi, slideKd);
        double slidePid = slideController.calculate(slidePos, slideTarget);
//        armAngle= armTarget/armTicks_in_degree-85;
//      double ff = Math.cos(Math.toRadians(armAngle)* armKf);
        double slideff = Math.sin(armAngle*slideKf);
        double slidePower = slidePid + slideff;
        slide.setPower(slidePower);

        telemetry.addData("armPos",armPos);
        telemetry.addData("armTarget",armTarget);
        telemetry.addData("armAngle",armAngle);
        telemetry.addData("armff",armff);
        telemetry.addData("armPid",armPid);
        telemetry.addData("armPower",arm.getPower());
        telemetry.addData("slidePos",slidePos);
        telemetry.addData("slideTarget",slideTarget);

    }
}
