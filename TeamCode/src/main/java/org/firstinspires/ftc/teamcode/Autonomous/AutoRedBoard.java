

package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name="AutoRedBoard", group="Linear OpMode")

public class AutoRedBoard extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//     private DcMotorEx flMotor = null;
//     private DcMotorEx frMotor = null;
//     private DcMotorEx blMotor = null;
//     private DcMotorEx brMotor = null;

    public OpenCvWebcam webcam;
    boolean Left= false;
    boolean Right = false;
    boolean Center = false;
    private Servo drop = null;
    private Servo wrist = null;
    private Servo guard = null;
    private DcMotorEx arm =null;



    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam  "), cameraMonitorViewId);
        webcam.setPipeline(new AutoRedBoard.samplePipeline());
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        drop =hardwareMap.get(Servo.class, "drop");
        arm =hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        guard = hardwareMap.get(Servo.class, "wrist2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.update();
        int count=0;
         /*
          Initialize the hardware variables. Note that the strings used here as parameters
          to 'get' must correspond to the names assigned during the robot configuration
          step (using the FTC Robot Controller app on the phone).
         */

        initializeDriveMotor();
//        frMotor.setTargetPosition(0);
//        flMotor.setTargetPosition(0);
//        brMotor.setTargetPosition(0);
//        blMotor.setTargetPosition(0);
//
//        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        frMotor.setPower(.3);
//        flMotor.setPower(.3);
//        brMotor.setPower(.3);
//        blMotor.setPower(.3);
        wrist.setPosition(.488);
        drop.setPosition(.01);
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left sti ck forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }
            sleep(100);
            while(count==0){
                sleep(1000);
                if(Left){
                    encoderDrive(FORWARD,DRIVE_SPEED,24,24,500);
                    sleep(500);
                    encoderDrive(LEFTSLIDE,DRIVE_SPEED,6,6,500);
                    sleep(800);
                    drop.setPosition(.4);
                    sleep(800);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,39,39,500);
                    sleep(500);
                    encoderDrive(RIGHT,DRIVE_SPEED,24,24,500);
                    sleep(500);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,8.5,8.5,500);
                    sleep(500);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1300);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1301);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                    wrist.setPosition(.878333333334);
                    sleep(500);
                    encoderDrive(BACKWARD,DRIVE_SPEED,15,15,500);
                    sleep(500);
                    guard.setPosition(.2);
                    sleep(1000);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1900);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1901);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    encoderDrive(FORWARD,DRIVE_SPEED,5,5,500);
                    sleep(500);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,20,20,500);
                    wrist.setPosition(.488);
                    sleep(500);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-500);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-501);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(1500);
                    encoderDrive(BACKWARD,DRIVE_SPEED,11,11,500);
//                    sleep(1500);
//                    sleep(2000);
//                    base.turnLeft(1900,.3);
//                    sleep(1500);
//                    base.driveRight(1725,.3);
//                    sleep(1200);
//                    base.arm.setPower(-.3);
//                    sleep(1200);
//                    base.arm.setPower(0);
//                    sleep(1000);
//                    base.driveBack(540,.3);
//                    base.wrist.setPosition(1);
//                    sleep(2000);
//                    base.guard.setPosition(1);

                }
                else if(Right){
                    encoderDrive(FORWARD,DRIVE_SPEED,25,25,500);
                    sleep(500);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,18,18,500);
                    sleep(800);
                    drop.setPosition(.5);
                    sleep(800);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,17,17,500);
                    sleep(500);
                    encoderDrive(RIGHT,DRIVE_SPEED,24,24,500);
                    sleep(500);
                    encoderDrive(LEFTSLIDE,DRIVE_SPEED,7,7,500);
                    sleep(500);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1300);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1301);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    wrist.setPosition(.878333333334);
                    sleep(1000);
                    encoderDrive(BACKWARD,DRIVE_SPEED,13,13,500);
                    sleep(500);
                    guard.setPosition(.2);
                    sleep(1000);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1900);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1901);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(2000);
                    encoderDrive(FORWARD,DRIVE_SPEED,5,5,500);
                    sleep(500);
                    encoderDrive(RIGHTSLIDE, DRIVE_SPEED, 38, 38, 500);

                    wrist.setPosition(.488);
                    sleep(500);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-500);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-501);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(1500);
                    encoderDrive(BACKWARD,DRIVE_SPEED,11,11,500);
//                    sleep(1500);
//                    base.driveLeft(300,.3);
//                    sleep(1200);
//                    base.arm.setPower(-.5);
//                    sleep(1700);
//                    base.arm.setPower(0);
//                    sleep(1000);
//                    base.driveBack(1150,.3);
//                    sleep(5000);
//                    base.wrist.setPosition(1);
//                    sleep(1000);
//                    base.guard.setPosition(1);
                }
                else if(Center){
                    encoderDrive(FORWARD,DRIVE_SPEED,25,25,500);
                    sleep(800);
                    drop.setPosition(.4);
                    sleep(800);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,39,39,500);
                    sleep(500);
                    encoderDrive(RIGHT,DRIVE_SPEED,24,24,500);
                    sleep(500);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1300);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1301);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    wrist.setPosition(.878333333334);
                    sleep(500);
                    encoderDrive(BACKWARD,DRIVE_SPEED,9,9,500);
                    sleep(500);
                    guard.setPosition(.2);
                    sleep(1000);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1900);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-1901);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(1000);
                    encoderDrive(FORWARD,DRIVE_SPEED,5,5,500);
                    sleep(500);
                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,27,27,500);

                    wrist.setPosition(.488);
                    sleep(500);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-500);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(3000);
                    arm.setTargetPosition(-501);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    sleep(1500);
                    encoderDrive(BACKWARD,DRIVE_SPEED,11,11,500);
//                    sleep(1500);




//                    encoderDrive(RIGHTSLIDE,DRIVE_SPEED,10,10,500);
//                    sleep(1200);
//                    base.arm.setPower(-.5);
//                    sleep(1700);
//                    base.arm.setPower(0);
//                    sleep(1000);
//                    base.driveBack(800,.3);
//                    base.wrist.setPosition(1);
//                    sleep(2000);
//                    base.guard.setPosition(1);
                }

                count+=1;
            }




            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.update();
        }
    }
    class samplePipeline extends OpenCvPipeline {

        Cam2 web = new Cam2();
        boolean viewportPaused;
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat centerCrop;
        double leftavgfin;
        double centeravgfin;
        double rightavgfin;

        Mat output = new Mat();
        Scalar rectColor = new Scalar(255, 0, 0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running╰(*°▽°*)╯╰(*°▽°*)╯");
            telemetry.update();
            Rect leftrect = new Rect(5, 200, 10, 10);
            Rect centerrect = new Rect(315, 170, 10, 10);
            Rect rightrect = new Rect(610, 210, 10, 10);

            input.copyTo(output);
            Imgproc.rectangle(output, leftrect, rectColor, 2);
            Imgproc.rectangle(output, centerrect, new Scalar(0,255,0), 2);
            Imgproc.rectangle(output, rightrect, new Scalar(0,0,255), 2);

            leftCrop = YCbCr.submat(leftrect);
            centerCrop = YCbCr.submat(centerrect);
            rightCrop = YCbCr.submat(rightrect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(centerCrop, centerCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar centeravg = Core.mean(centerCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            centeravgfin = centeravg.val[0];
            rightavgfin = rightavg.val[0];
            telemetry.addData("Left", leftavgfin);
            telemetry.addData("Center", centeravgfin);
            telemetry.addData("Right", rightavgfin);

            if (leftavgfin < rightavgfin && leftavgfin<centeravgfin) {
                telemetry.addLine("Left");
                Left=true;
                Right=false;
                Center=false;


                telemetry.update();
            }
            else if (leftavgfin > rightavgfin&& rightavgfin<centeravgfin) {
                telemetry.addLine("Right");
                Left=false;
                Right=true;
                Center=false;


                telemetry.update();
            }
            else if (leftavgfin > centeravgfin && rightavgfin>centeravgfin) {
                telemetry.addLine("Center");
                Left=false;
                Right=false;
                Center=true;


                telemetry.update();            }

            return (output);


        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                web.webcam.pauseViewport();
            } else {
                web.webcam.resumeViewport();
            }
        }
    }
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

    private static final double DRIVE_SPEED = .5;
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
            telemetry.addLine("reset");
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = blMotor.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newLeftFrontTarget = flMotor.getCurrentPosition() + (int)(leftInches * Math.abs(COUNTS_PER_INCH));
            newRightBackTarget = brMotor.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));
            newRightFrontTarget = frMotor.getCurrentPosition() + (int)(rightInches * Math.abs(COUNTS_PER_INCH));

            runtime.reset();
            switch (direction) {
                case RIGHT:
                    blMotor.setTargetPosition(newLeftBackTarget );
                    brMotor.setTargetPosition(-newRightBackTarget);
                    flMotor.setTargetPosition(newLeftFrontTarget );
                    frMotor.setTargetPosition(-newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case LEFT:
                    blMotor.setTargetPosition(-newLeftBackTarget);
                    brMotor.setTargetPosition(newRightBackTarget);
                    flMotor.setTargetPosition(-newLeftFrontTarget);
                    frMotor.setTargetPosition(newRightFrontTarget);

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
                    blMotor.setTargetPosition(-newLeftBackTarget);
                    brMotor.setTargetPosition(newRightBackTarget );
                    flMotor.setTargetPosition(newLeftFrontTarget );
                    frMotor.setTargetPosition(-newRightFrontTarget);

//                    leftBackDrive.setPower(Math.abs(speed));
//                    rightBackDrive.setPower(Math.abs(speed));
//                    leftFrontDrive.setPower(Math.abs(speed));
//                    rightFrontDrive.setPower(Math.abs(speed));

                    break;
                case RIGHTSLIDE:
                    blMotor.setTargetPosition(newLeftBackTarget );
                    brMotor.setTargetPosition(-newRightBackTarget);
                    flMotor.setTargetPosition(-newLeftFrontTarget);
                    frMotor.setTargetPosition(newRightFrontTarget );

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

//              sleep(500);   // optional pause after each move
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


}
