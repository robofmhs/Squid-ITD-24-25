
package org.firstinspires.ftc.teamcode.AutonomousV1;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;





public class RobotBaseMovementService extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor flMotor ;
    private DcMotor frMotor ;
    private DcMotor blMotor ;
    private DcMotor brMotor ;



    private static final double COUNTS_PER_MOTOR_REV = 535;    // eg: TETRIX Motor Encoder 20:1
    private static final double WHEEL_DIAMETER_INCHES = 3.85;     // For figuring circumference
    //    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double DRIVE_SPEED = .3;
    private static final String LEFT = "LEFT";
    private static final String RIGHT = "RIGHT";
    private static final String FORWARD = "FORWARD";
    private static final String BACKWARD = "BACKWARD";
    private static final String LEFTSLIDE = "LEFTSLIDE";
    private static final String RIGHTSLIDE = "RIGHTSLIDE";






    public void encoderDrive(String direction, double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;


        if (opModeIsActive()) {

            //reset all encoder values
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = blMotor.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newLeftFrontTarget = flMotor.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newRightBackTarget = brMotor.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));
            newRightFrontTarget = frMotor.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));

            runtime.reset();
            switch (direction) {
                case RIGHT:
                    blMotor.setTargetPosition(-newLeftBackTarget );
                    brMotor.setTargetPosition(newRightBackTarget);
                    flMotor.setTargetPosition(-newLeftFrontTarget );
                    frMotor.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFT:
                    blMotor.setTargetPosition(newLeftBackTarget);
                    brMotor.setTargetPosition(-newRightBackTarget);
                    flMotor.setTargetPosition(newLeftFrontTarget);
                    frMotor.setTargetPosition(-newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case BACKWARD:
                    blMotor.setTargetPosition(newLeftBackTarget);
                    brMotor.setTargetPosition(newRightBackTarget);
                    flMotor.setTargetPosition(newLeftFrontTarget);
                    frMotor.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;

                case FORWARD:
                    blMotor.setTargetPosition(-newLeftBackTarget );
                    brMotor.setTargetPosition(-newRightBackTarget );
                    flMotor.setTargetPosition(-newLeftFrontTarget );
                    frMotor.setTargetPosition(-newRightFrontTarget );

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFTSLIDE:
                    blMotor.setTargetPosition(newLeftBackTarget);
                    brMotor.setTargetPosition(-newRightBackTarget );
                    flMotor.setTargetPosition(-newLeftFrontTarget );
                    frMotor.setTargetPosition(newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case RIGHTSLIDE:
                    blMotor.setTargetPosition(-newLeftBackTarget );
                    brMotor.setTargetPosition(newRightBackTarget);
                    flMotor.setTargetPosition(newLeftFrontTarget);
                    frMotor.setTargetPosition(-newRightFrontTarget );

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
            }


            // Turn On RUN_TO_POSITION
            blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            blMotor.setPower(Math.abs(speed));
            brMotor.setPower(Math.abs(speed));
            flMotor.setPower(Math.abs(speed));
            frMotor.setPower(Math.abs(speed));

//              sleep(5000);   // optional pause after each move
//
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (blMotor.isBusy() && brMotor.isBusy() && flMotor.isBusy() && frMotor.isBusy())) {
                telemetry.addData("isbusy", "right back and left back" + brMotor.isBusy() + " " + blMotor.isBusy());
                telemetry.addData("Target", "Back Running to %7d :%7d", newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Back Current", "Back Running at %7d :%7d", blMotor.getCurrentPosition(), brMotor.getCurrentPosition());
                telemetry.addData("Back Target", "Back Running at %7d :%7d", blMotor.getTargetPosition(), brMotor.getTargetPosition());
                telemetry.addData("Target", "Front Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Front Current", "Front Running at %7d :%7d", flMotor.getCurrentPosition(), frMotor.getCurrentPosition());
                telemetry.addData("Front Target", "Back Running at %7d :%7d", flMotor.getTargetPosition(), frMotor.getTargetPosition());
                telemetry.addData("Speed", "right back and left back " + brMotor.getPower() + " " + blMotor.getPower());
                telemetry.addData("Speed", "right back and left back " + frMotor.getPower() + " " + brMotor.getPower());
                telemetry.addData("Speed", "Direction " + direction);
                telemetry.update();
            }

            // Stop all motion;
            flMotor.setPower(0);
            blMotor.setPower(0);
            frMotor.setPower(0);
            brMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void initializeDriveMotor() {
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        flMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn off RUN_TO_POSITION
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    @Override
    public void runOpMode() throws InterruptedException {
    }
}