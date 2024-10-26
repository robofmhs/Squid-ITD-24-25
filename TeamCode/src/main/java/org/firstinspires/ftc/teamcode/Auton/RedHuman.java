

package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp
public class RedHuman extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime auton = new ElapsedTime();

    //arm stuff
    private PIDController armController;

    public static double armKp = 0.015, armKi = 0.0005, armKd =0.0005;
    public static double armIncrement = 10, slideIncrement=10;
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

    public static double gOpen = .75;
    public static double gClose = 1;
//    public static double wristPos = .8;

    private final double ticks_in_degree = 5281.1/360;

    private DcMotorEx flMotor ;
    private DcMotorEx frMotor ;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor ;

    private Servo Gripper;

    private Servo Wrist;


    public void gripper(Servo servo1, double pos1,double pos2){
        if (servo1.getPosition() >= pos1 && servo1.getPosition() < pos1+.01) {
            servo1.setPosition(pos2);

        } else if (servo1.getPosition() >= pos2) {
            servo1.setPosition(pos1);
        } else {
            servo1.setPosition(pos2);
        }
    }
    InterpLUT lut = new InterpLUT();
    InterpLUT slideLimit = new InterpLUT();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        flMotor= hardwareMap.get(DcMotorEx.class,"fl");
        frMotor= hardwareMap.get(DcMotorEx.class,"fr");
        blMotor= hardwareMap.get(DcMotorEx.class,"bl");
        brMotor= hardwareMap.get(DcMotorEx.class,"br");
        Gripper = hardwareMap.get(Servo.class,"gl"); // is port 2 in expansion
        Wrist= hardwareMap.get(Servo.class,"wrist"); // is port 0 in expansion
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Wrist.setPosition(0);
        frMotor.setDirection(DcMotorEx.Direction.REVERSE);
        brMotor.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
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
        //slideLimit input armPos, output slideLimit;
        slideLimit.add(0,0);
        slideLimit.add(100,0);
        slideLimit.add(200,0);
        slideLimit.add(300,0);
        slideLimit.add(400,0);
        slideLimit.add(500,0);
        slideLimit.add(600,0);
        slideLimit.add(700,0);
        slideLimit.add(800,0);
        slideLimit.add(900,0);
        slideLimit.add(1000,0);
        slideLimit.add(1100,0);
        slideLimit.add(1200,0);
        slideLimit.add(1300,0);
        slideLimit.add(1400,0);
        slideLimit.add(1500,0);
        slideLimit.add(1600,0);
        slideLimit.add(1700,0);
        slideLimit.add(1800,0);
        slideLimit.add(1900,0);
        slideLimit.add(2000,0);
        slideLimit.add(2100,0);
        slideLimit.add(2200,0);
        slideLimit.add(2300,0);
        slideLimit.add(2400,0);
        slideLimit.add(2500,0);
        slideLimit.add(2600,0);
        slideLimit.add(2700,0);
        slideLimit.add(2800,0);
        slideLimit.add(2900,0);
        slideLimit.add(3000,0);
        slideLimit.add(3100,0);
        slideLimit.add(3200,0);
        slideLimit.add(3300,0);
        slideLimit.add(3400,0);
        slideLimit.add(3500,0);
        slideLimit.add(3700,0);
        slideLimit.add(3800,0);
        slideLimit.add(3900,0);
        slideLimit.add(4000,0);
        slideLimit.createLUT();
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        auton.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int armPos = arm.getCurrentPosition();
            int slidePos = Range.clip(slide.getCurrentPosition(),1,1000);
            slideTarget= Range.clip(slideTarget,1,1000);
            armTarget= Range.clip(armTarget,0,2400);
            armController.setPID(armKp, armKi, armKd);

            double armPid = armController.calculate(armPos, armTarget);

            armAngle= armPos/armTicks_in_degree-65;
            armKf=lut.get(slidePos);
            double armff = Math.cos(Math.toRadians(armAngle))*armKf;
//        double ff = Math.sin(Math.toRadians(armPos/armTicks_in_degree+5)*armKf);
            double armPower = armPid + armff;
            arm.setPower(armPower);

            slideController.setPID(slideKp, slideKi, slideKd);
            double slidePid = slideController.calculate(slidePos, slideTarget);
            double slideff = Math.sin(armAngle*slideKf);
            double slidePower = slidePid + slideff;
            slide.setPower(slidePower);
        }
    }
    private void move(String direction, double speed,long duration){
        auton.reset();
        if(direction.equals("forward")){
            flMotor.setPower(speed);
            frMotor.setPower(speed);
            blMotor.setPower(speed);
            brMotor.setPower(speed);
        } else if (direction.equals("back")) {
            flMotor.setPower(-speed);
            frMotor.setPower(-speed);
            blMotor.setPower(-speed);
            brMotor.setPower(-speed);
        } else if (direction.equals("leftStrafe")) {
            flMotor.setPower(-speed);
            frMotor.setPower(speed);
            blMotor.setPower(speed);
            brMotor.setPower(-speed);
        } else if (direction.equals("rightStrafe")) {
            flMotor.setPower(speed);
            frMotor.setPower(-speed);
            blMotor.setPower(-speed);
            brMotor.setPower(speed);
        } else if (direction.equals("rightTurn")) {
            flMotor.setPower(speed);
            frMotor.setPower(speed);
            blMotor.setPower(-speed);
            brMotor.setPower(-speed);
        } else if (direction.equals("leftTurn")) {
            flMotor.setPower(speed);
            frMotor.setPower(speed);
            blMotor.setPower(-speed);
            brMotor.setPower(-speed);
        }
        if(auton.seconds()>=duration){
            flMotor.setPower(0);
            frMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
        }

    }
}