

package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.Base;
import org.firstinspires.ftc.teamcode.Constants.Constants;


@Config
@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class RC_Base extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private org.firstinspires.ftc.teamcode.Constants.Base Base;
    private org.firstinspires.ftc.teamcode.Constants.Constants Constants;
    private PIDF_Arm armFunc = new PIDF_Arm();

    private PIDController controller;

    public static double p = 0.03,i = 0,d =0.0008;
    public static double f = 0.0001;
    public static int target = 0;

    private final double ticks_in_degree = 5281.1/360;


    private DcMotorEx flMotor ;
    private DcMotorEx frMotor ;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor ;
    private DcMotorEx arm;
    private Servo GripperLeft;
    private Servo GripperRight;
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
    public void gripper2(Servo servo1, double pos1,double pos2,Servo servo2, double pos12,double pos22){
        this.gripper(servo1,pos1,pos2);
        this.gripper(servo2,pos12,pos22);
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
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flMotor= hardwareMap.get(DcMotorEx.class,"fl");
        frMotor= hardwareMap.get(DcMotorEx.class,"fr");
        blMotor= hardwareMap.get(DcMotorEx.class,"bl");
        brMotor= hardwareMap.get(DcMotorEx.class,"br");
         arm= hardwareMap.get(DcMotorEx.class,"arm"); // is port 0 on expansion
        GripperLeft= hardwareMap.get(Servo.class,"gl"); // is port 2 in expansion
        GripperRight= hardwareMap.get(Servo.class,"gr"); // is port 1 in expansion
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

            if(gamepad2.a){
                GripperLeft.setPosition(0.3);
            }
            if(gamepad2.b){
                GripperRight.setPosition(0.8);
            }

            if(gamepad1.a){
                gripper2(GripperLeft,.3,
                        .5,GripperRight,
                        .8, .5);
                while (gamepad1.a){}
            }

//            if (gamepad1.left_trigger>0 && gamepad1.left_bumper) {
//                arm.setVelocity(4000);
//                arm.setTargetPosition(arm.getCurrentPosition() + 100);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);               }
//            else if (gamepad1.right_trigger>0 && gamepad1.left_bumper) {
//                arm.setVelocity(4000);
//                arm.setTargetPosition(arm.getCurrentPosition() - 100);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);               }
//            else if(gamepad1.left_trigger>0){
//                arm.setVelocity(4000);
//                arm.setTargetPosition(arm.getCurrentPosition() + 200);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);            }
//            else if (gamepad1.right_trigger>0) {
//                arm.setVelocity(4000);
//                arm.setTargetPosition(arm.getCurrentPosition() - 200);
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);            }
//            else{
//                arm.setVelocity(0);
//                arm.setTargetPosition(arm.getCurrentPosition());
//                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);            }



            if(gamepad1.dpad_up){
                Wrist.setPosition(Wrist.getPosition()+.001);
            }
            else if(gamepad1.dpad_down){
                Wrist.setPosition(Wrist.getPosition()-.001);
            } else {
                Wrist.setPosition(Wrist.getPosition());
            }

            controller.setPID(p,i,d);
            int armPos = arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)*f);
            double ff = Math.sin(Math.toRadians(armPos/ticks_in_degree+30)*f);
            double power = pid +ff;
//            arm.setPower(power);

            ExternalArm(target,arm);

            telemetry.addData("pos: ", armPos);
            telemetry.addData("target: ", target);
            telemetry.addData("power: ", power);
            telemetry.update();            /*
            set pos 1:
                arm - 356
                wrist - .4456
            set pos 2:
                arm -
                wrist -
             */
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("arm: ", arm.getCurrentPosition());
            telemetry.addData("Grippers", "left (%.2f), right (%.2f)", GripperLeft.getPosition(), GripperRight.getPosition());
            telemetry.addData("wrist", Wrist.getPosition());
            telemetry.update();
        }
    }
}
