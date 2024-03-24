
package org.firstinspires.ftc.teamcode.AutonomousV1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name="test", group="LinearOpMode")

public class Test extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;



    static final double COUNTS_PER_MOTOR_REV = 535;    // eg: TETRIX Motor Encoder 20:1
    static final double WHEEL_DIAMETER_INCHES = 3.85;     // For figuring circumference
    //    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = .3;
    static final String LEFT = "LEFT";
    static final String RIGHT = "RIGHT";
    static final String FORWARD = "FORWARD";
    static final String BACKWARD = "BACKWARD";
    static final String LEFTSLIDE = "LEFTSLIDE";
    static final String RIGHTSLIDE = "RIGHTSLIDE";






    public void encoderDrive(String direction, double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;


        if (opModeIsActive()) {

            //reset all encoder values
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));

            runtime.reset();
            switch (direction) {
                case RIGHT:
                    leftBackDrive.setTargetPosition(-newLeftBackTarget );
                    rightBackDrive.setTargetPosition(newRightBackTarget);
                    leftFrontDrive.setTargetPosition(-newLeftFrontTarget );
                    rightFrontDrive.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFT:
                    leftBackDrive.setTargetPosition(newLeftBackTarget);
                    rightBackDrive.setTargetPosition(-newRightBackTarget);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                    rightFrontDrive.setTargetPosition(-newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case BACKWARD:
                    leftBackDrive.setTargetPosition(newLeftBackTarget);
                    rightBackDrive.setTargetPosition(newRightBackTarget);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                    rightFrontDrive.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;

                case FORWARD:
                    leftBackDrive.setTargetPosition(-newLeftBackTarget );
                    rightBackDrive.setTargetPosition(-newRightBackTarget );
                    leftFrontDrive.setTargetPosition(-newLeftFrontTarget );
                    rightFrontDrive.setTargetPosition(-newRightFrontTarget );

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFTSLIDE:
                    leftBackDrive.setTargetPosition(newLeftBackTarget);
                    rightBackDrive.setTargetPosition(-newRightBackTarget );
                    leftFrontDrive.setTargetPosition(-newLeftFrontTarget );
                    rightFrontDrive.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case RIGHTSLIDE:
                    leftBackDrive.setTargetPosition(-newLeftBackTarget );
                    rightBackDrive.setTargetPosition(newRightBackTarget);
                    leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                    rightFrontDrive.setTargetPosition(-newRightFrontTarget );

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
            }


            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));

//              sleep(5000);   // optional pause after each move
//
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy() && leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
                telemetry.addData("isbusy", "right back and left back" + rightBackDrive.isBusy() + " " + leftBackDrive.isBusy());
                telemetry.addData("Target", "Back Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Back Current", "Back Running at %7d :%7d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Back Target", "Back Running at %7d :%7d", leftBackDrive.getTargetPosition(), rightBackDrive.getTargetPosition());
                telemetry.addData("Target", "Front Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Front Current", "Front Running at %7d :%7d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Front Target", "Back Running at %7d :%7d", leftFrontDrive.getTargetPosition(), rightFrontDrive.getTargetPosition());
                telemetry.addData("Speed", "right back and left back " + rightBackDrive.getPower() + " " + leftBackDrive.getPower());
                telemetry.addData("Speed", "right back and left back " + rightFrontDrive.getPower() + " " + rightBackDrive.getPower());
                telemetry.addData("Speed", "Direction " + direction);
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //  sleep(250);   // optional pause after each move
        }
    }


    private void initializeDriveMotor() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    @Override

    public void runOpMode() throws InterruptedException {
        initializeDriveMotor();
        waitForStart();
        while (opModeIsActive()){
        encoderDrive(FORWARD,DRIVE_SPEED,10,10,20);
        sleep(200);
        encoderDrive(LEFTSLIDE,DRIVE_SPEED,24,24,20);
        }
    }
}