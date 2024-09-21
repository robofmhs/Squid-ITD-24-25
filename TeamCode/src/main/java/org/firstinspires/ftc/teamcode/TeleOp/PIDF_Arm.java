package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_Arm extends OpMode{
    private PIDController controller;

    public static double p = 0.01,i = 0,d =0.001;
    public static double f = 0.0001;
    public static int target = 0;

    private final double ticks_in_degree = 5281.1/360;

    private DcMotorEx arm;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");

    }
    public void ExternalArm(int target,DcMotorEx arm){
        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)*f);
        double ff = Math.sin(Math.toRadians(armPos/ticks_in_degree+30)*f);
        double power = pid +ff;
        arm.setPower(power);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)*f);
        double ff = Math.sin(Math.toRadians(armPos/ticks_in_degree+30)*f);
        double power = pid +ff;
        arm.setPower(power);
//        ExternalArm(target,arm);
        telemetry.addData("pos: ", armPos);
        telemetry.addData("target: ", target);
        telemetry.addData("power: ", power);
        telemetry.update();
    }
}
