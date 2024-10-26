package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DiscordHelp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        Servo rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        CRServo activeIntake = hardwareMap.get(CRServo.class, "activeIntake");
        DcMotor intakeArmMotor = hardwareMap.get(DcMotor.class, "intakeArmMotor");
        DcMotor hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        Servo bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setTargetPosition(0);
        intakeArmMotor.setPower(0);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go for   wards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double rightSlideMotorDown = gamepad2.left_stick_y * -1;
            double rightSlideMotorUp = gamepad2.left_stick_y;
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.y) {
                sleep(1000);
                intakeArmMotor.setTargetPosition(10);
                sleep(1000);
                rightWristServo.setPosition(0.3);
                sleep(1000);
                bucketServo.setPosition(0);
            }
            if (gamepad1.a) {
                sleep(1000);
                rightWristServo.setPosition(0.5);
                sleep(1000);
                intakeArmMotor.setTargetPosition(-10);
                sleep(1000);
            }
            if (gamepad2.y) {
                bucketServo.setPosition(0.1);
            }
            if (gamepad2.a) {
                bucketServo.setPosition(0.8);
            }
            if (gamepad1.right_bumper) {
                activeIntake.setPower(-1);
            }
            if (gamepad1.left_bumper) {
                activeIntake.setPower(0);
            }
            if (gamepad1.y) {
                activeIntake.setPower(1);
            }
            if (gamepad2.left_stick_y == 1) {
                rightSlideMotor.setPower(0.5);
            }
            if (gamepad2.left_stick_y == -1) {
                rightSlideMotor.setPower(0.5);
            }
            //  if (gamepad2.x) {
            //    hangMotor.setPower(0.5);
            //}
            //     if (gamepad2.b) {
            //    hangMotor.setPower(-0.5);
            //  }
        }
    }
}
