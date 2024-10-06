package org.firstinspires.ftc.teamcode.Auton;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedHuman extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // motors
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    private DcMotorEx arm;

    private Servo wrist;
    private Servo gripper;

    private PIDController controller;

    private double depotPos=.3;
    private double pickupPos=.8;
    public static double gOpen = .5;
    public static double gClose = .0;



    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initializeDriveMotor();
        arm= hardwareMap.get(DcMotorEx.class,"arm");
        wrist= hardwareMap.get(Servo.class,"wrist");
        gripper = hardwareMap.get(Servo.class,"gripper");
        controller = new PIDController(p, i, d);



        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            encoderDrive(FORWARD, DRIVE_SPEED, 12, 12, 750);
            encoderDrive(LEFTSLIDE, DRIVE_SPEED, 12, 12, 750);
            ExternalArm(900);
            wrist.setPosition(depotPos);
            ExternalArm(600);
            sleep(700);
            gripper.setPosition(gOpen);
            sleep(700);
            encoderDrive(BACKWARD, DRIVE_SPEED, 12, 12, 750);
            encoderDrive(RIGHTSLIDE, DRIVE_SPEED, 24, 24, 750);
            encoderDrive(FORWARD, DRIVE_SPEED, 12, 12, 750);
            wrist.setPosition(pickupPos);
            sleep(700);
            ExternalArm(200);
            encoderDrive(FORWARD, .3, 3, 3, 750);
            gripper.setPosition(gClose);
            sleep(700);
            encoderDrive(BACKWARD, DRIVE_SPEED, 6, 6, 750);
            ExternalArm(2500);
            gripper.setPosition(gOpen);
            encoderDrive(FORWARD, DRIVE_SPEED, 6, 6, 750);
            encoderDrive(RIGHTSLIDE, DRIVE_SPEED, 4, 4, 750);
            ExternalArm(200);
            encoderDrive(FORWARD, .3, 3, 3, 750);
            gripper.setPosition(gClose);
            ExternalArm(2500);
            encoderDrive(BACKWARD, DRIVE_SPEED, 6, 6, 750);
            gripper.setPosition(gOpen);
            encoderDrive(FORWARD, DRIVE_SPEED, 6, 6, 750);
            encoderDrive(RIGHTSLIDE, DRIVE_SPEED, 4, 4, 750);
            ExternalArm(200);
            encoderDrive(FORWARD, .3, 3, 3, 750);
            gripper.setPosition(gClose);
            ExternalArm(2500);
            encoderDrive(BACKWARD, DRIVE_SPEED, 6, 6, 750);
            gripper.setPosition(gOpen);
            ExternalArm(200);

            encoderDrive(FORWARD, DRIVE_SPEED, 6, 6, 750);
            encoderDrive(LEFTSLIDE, DRIVE_SPEED, 12, 12, 750);
            encoderDrive(LEFT, DRIVE_SPEED, 90, 90, 750);
            encoderDrive(FORWARD, DRIVE_SPEED, 6, 6, 750);
            gripper.setPosition(gClose);
            ExternalArm(600);
            wrist.setPosition(depotPos);
            encoderDrive(RIGHTSLIDE, DRIVE_SPEED, 12, 12, 750);
            encoderDrive(BACKWARD, DRIVE_SPEED, 12, 12, 750);
            ExternalArm(900);
            gripper.setPosition(gOpen);
            encoderDrive(FORWARD, DRIVE_SPEED, 16, 16, 750);
            encoderDrive(LEFTSLIDE, DRIVE_SPEED, 12, 12, 750);



        }

    }
    private static final double COUNTS_PER_MOTOR_REV = 435;    // eg: TETRIX Motor Encoder 20:1
    private static final double WHEEL_DIAMETER_INCHES = 3.85;     // For figuring circumference
    //    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double DRIVE_SPEED = .5;
    private static final String LEFT = "LEFT";
    private static final String RIGHT = "RIGHT";
    private static final String FORWARD = "FORWARD";
    private static final String BACKWARD = "BACKWARD";
    private static final String LEFTSLIDE = "LEFTSLIDE";
    private static final String RIGHTSLIDE = "RIGHTSLIDE";

    public static double p = 0.03,i = 0,d =0.0008;
    public static double f = 0.0001;

    private final double ticks_in_degree = 5281.1/360;

    public void ExternalArm(int target){
        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(armTarget/ticks_in_degree)*armKf);
        double ff = Math.sin(Math.toRadians(armPos/ticks_in_degree+30)*f);
        double power = pid +ff;
        arm.setPower(power);
    }

    public void encoderDrive(String direction, double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;


        if (opModeIsActive()) {

            //reset all encoder values
            bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("reset");
            // Determine new armTarget position, and pass to motor controller
            newLeftBackTarget = bl.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newLeftFrontTarget = fl.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newRightBackTarget = br.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));
            newRightFrontTarget = fr.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));

            runtime.reset();
            switch (direction) {
                case RIGHT:
                    bl.setTargetPosition(newLeftBackTarget );
                    br.setTargetPosition(-newRightBackTarget);
                    fl.setTargetPosition(newLeftFrontTarget );
                    fr.setTargetPosition(-newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFT:
                    bl.setTargetPosition(-newLeftBackTarget);
                    br.setTargetPosition(newRightBackTarget);
                    fl.setTargetPosition(-newLeftFrontTarget);
                    fr.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case BACKWARD:
                    bl.setTargetPosition(newLeftBackTarget);
                    br.setTargetPosition(newRightBackTarget);
                    fl.setTargetPosition(newLeftFrontTarget);
                    fr.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;

                case FORWARD:
                    bl.setTargetPosition(-newLeftBackTarget );
                    br.setTargetPosition(-newRightBackTarget );
                    fl.setTargetPosition(-newLeftFrontTarget );
                    fr.setTargetPosition(-newRightFrontTarget );

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFTSLIDE:
                    bl.setTargetPosition(-newLeftBackTarget);
                    br.setTargetPosition(newRightBackTarget );
                    fl.setTargetPosition(newLeftFrontTarget );
                    fr.setTargetPosition(-newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case RIGHTSLIDE:
                    bl.setTargetPosition(newLeftBackTarget );
                    br.setTargetPosition(-newRightBackTarget);
                    fl.setTargetPosition(-newLeftFrontTarget);
                    fr.setTargetPosition(newRightFrontTarget );

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
            }


            // Turn On RUN_TO_POSITION
            bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            bl.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));
            fl.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));

              sleep((long)(timeoutS));   // optional pause after each move
//

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its armTarget position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < (timeoutS*800000)) &&
                    (bl.isBusy() && br.isBusy() && fl.isBusy() && fr.isBusy())) {
                telemetry.addData("isbusy", "right back and left back" + br.isBusy() + " " + bl.isBusy());
                telemetry.addData("Target", "Back Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Back Current", "Back Running at %7d :%7d", bl.getCurrentPosition(), br.getCurrentPosition());
                telemetry.addData("Back Target", "Back Running at %7d :%7d", bl.getTargetPosition(), br.getTargetPosition());
                telemetry.addData("Target", "Front Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Front Current", "Front Running at %7d :%7d", fl.getCurrentPosition(), fr.getCurrentPosition());
                telemetry.addData("Front Target", "Back Running at %7d :%7d", fl.getTargetPosition(), fr.getTargetPosition());
                telemetry.addData("Speed", "right back and left back " + br.getPower() + " " + bl.getPower());
                telemetry.addData("Speed", "right back and left back " + fr.getPower() + " " + br.getPower());
                telemetry.addData("Speed", "Direction " + direction);
                telemetry.update();
            }

            // Stop all motion;
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

            // Turn off RUN_TO_POSITION
            bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//            sleep((long) timeoutS);   // optional pause after each move
        }
    }


    public void initializeDriveMotor() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }
}
