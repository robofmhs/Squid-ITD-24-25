

package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp
public class RC_Base extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private PIDController controller;

    public static double p = 0.03,i = 0,d =0.0008;
    public static double f = 0.0001;
    public static int target = 0;
    public static int increment = 1;
    public static double gOpen = .3;
    public static double gClose = .8;
    public static double wristPos = .8;

    private final double ticks_in_degree = 5281.1/360;


    private DcMotorEx flMotor ;
    private DcMotorEx frMotor ;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor ;
    private DcMotorEx drone ;
    private DcMotorEx arm;
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
    public void ExternalArm(int target,DcMotorEx arm){
        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(armTarget/ticks_in_degree)*armKf);
        double ff = Math.sin(Math.toRadians(armPos/ticks_in_degree+30)*f);
        double power = pid +ff;
        arm.setPower(power);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flMotor= hardwareMap.get(DcMotorEx.class,"fl");
        frMotor= hardwareMap.get(DcMotorEx.class,"fr");
        blMotor= hardwareMap.get(DcMotorEx.class,"bl");
        brMotor= hardwareMap.get(DcMotorEx.class,"br");
        drone= hardwareMap.get(DcMotorEx.class,"drone");
        arm= hardwareMap.get(DcMotorEx.class,"arm"); // is port 0 on expansion
        Gripper = hardwareMap.get(Servo.class,"gl"); // is port 2 in expansion
        Wrist= hardwareMap.get(Servo.class,"wrist"); // is port 0 in expansion

        Wrist.setPosition(1.0);
        frMotor.setDirection(DcMotorEx.Direction.REVERSE);
        brMotor.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                flMotor.setPower(frontLeftPower*0.35);
                blMotor.setPower(backLeftPower*0.35);
                frMotor.setPower(frontRightPower*0.35);
                brMotor.setPower(backRightPower*0.35);
            }
            else {
                flMotor.setPower(frontLeftPower);
                blMotor.setPower(backLeftPower);
                frMotor.setPower(frontRightPower);
                brMotor.setPower(backRightPower);
            }



            if(gamepad1.a){
                gripper(Gripper,gOpen,gClose);
                while (gamepad1.a){}
            }

            if (gamepad1.left_trigger>0 && gamepad1.left_bumper) {
                target+=increment;
                ExternalArm(target,arm);
            }
            else if (gamepad1.right_trigger>0 && gamepad1.left_bumper) {
                target-=increment;
                ExternalArm(target,arm);             }
            else if(gamepad1.left_trigger>0){
                target+=2*increment;
                ExternalArm(target,arm);            }
            else if (gamepad1.right_trigger>0) {
                target-=2*increment;
                ExternalArm(target,arm);           }
            else{
                ExternalArm(target,arm);
            }

            Wrist.setPosition(wristPos);

            if(gamepad1.dpad_up){
                wristPos=Wrist.getPosition()+.001;
                Wrist.setPosition(wristPos);
            }
            else if(gamepad1.dpad_down){
                wristPos=Wrist.getPosition()-.001;
                Wrist.setPosition(wristPos);
            } else {
                wristPos=Wrist.getPosition();
                Wrist.setPosition(wristPos);
            }

            controller.setPID(p,i,d);
            int armPos = arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(armTarget/ticks_in_degree)*armKf);
            double ff = Math.sin(Math.toRadians(armPos/ticks_in_degree+30)*f);
            double power = pid +ff;
//            arm.setPower(power);


            telemetry.addData("pos: ", armPos);
            telemetry.addData("armTarget: ", target);
            telemetry.addData("power: ", power);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Gripper: ",  Gripper.getPosition());
            telemetry.addData("wrist", Wrist.getPosition());
            telemetry.update();
        }
    }
}
