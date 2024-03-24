package org.firstinspires.ftc.teamcode.AutonomousV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

@Autonomous(name="RR_BlueStack", group="Linear OpMode")
public class RR_BlueStack extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public OpenCvWebcam webcam;
    boolean Left= false;
    boolean Right = false;
    boolean Center = false;
    private Servo drop =null;
    private Servo stack =null;
    private Servo wrist =null;
    private Servo guard =null;
    private DcMotorEx arm =null;
    private DcMotorEx intake =null;
    void setArmPos(int amount,int speed){
        arm.setTargetPosition(-amount);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(speed);
        arm.setTargetPosition(-(amount+1));
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    void dropArmPos(){
        arm.setTargetPosition(-1800);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(3000);
        arm.setTargetPosition(-1801);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(1000);
        wrist.setPosition(.86);
        sleep(500);
        guard.setPosition(.4);
        sleep(800);
        arm.setTargetPosition(-2100);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(3000);
        arm.setTargetPosition(-2101);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    void dropArmPos2(){
        wrist.setPosition(.49);
        arm.setTargetPosition(-2100);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(5000);
        arm.setTargetPosition(-2101);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(1000);
        wrist.setPosition(.85);
        sleep(500);
        guard.setPosition(.5);
        sleep(800);

    }
    void resetArmPos(){
        wrist.setPosition(.49);
        sleep(500);
        arm.setTargetPosition(-60);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(3000);
        arm.setTargetPosition(-61);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        drop.setPosition(.01);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drop = hardwareMap.get(Servo.class, "drop");
        stack = hardwareMap.get(Servo.class, "stack");
        wrist = hardwareMap.get(Servo.class, "wrist");
        guard = hardwareMap.get(Servo.class, "wrist2");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wrist.setPosition(.49);
        stack.setPosition(1);
        guard.setPosition(.7);
        drop.setPosition(.01);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam  "), cameraMonitorViewId);

        webcam.setPipeline(new RR_BlueStack.samplePipeline());

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

        if (isStopRequested()) return;

        int count=0;
        Pose2d startPose = new Pose2d(-36, (70-9), Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence RedBoardRight = getRedBoardRight(drive, startPose);


        TrajectorySequence RedBoardCenter = getRedBoardCenter(drive, startPose);

        TrajectorySequence RedBoardLeft = getRedBoardLeft(drive, startPose);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                /*
//                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
//                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
//                 * if the reason you wish to stop the stream early is to switch use of the camera
//                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
//                 * (commented out below), because according to the Android Camera API documentation:
//                 *         "Your application should only have one Camera object active at a time for
//                 *          a particular hardware camera."
//                 *
//                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
//                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
//                 *
//                 * NB2: if you are stopping the camera stream to simply save some processing power
//                 * (or battery power) for a short while when you do not need your vision pipeline,
//                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
//                 * it the next time you wish to activate your vision pipeline, which can take a bit of
//                 * time. Of course, this comment is irrelevant in light of the use case described in
//                 * the above "important note".
//                 */
//                webcam.stopStreaming();
////                //webcam.closeCameraDevice();
//            }
//            sleep(100);
            while(count==0){
                if(Left){
                    drive.followTrajectorySequence(RedBoardLeft);
                    webcam.closeCameraDevice();
                } else if(Right) {
                    drive.followTrajectorySequence(RedBoardRight);
                    webcam.closeCameraDevice();
                } else if(Center) {
                    drive.followTrajectorySequence(RedBoardCenter);
                    webcam.closeCameraDevice();
                }
                count+=1;
            }
        }



    }

//    private TrajectorySequence getRedBoardLeft(SampleMecanumDrive drive, Pose2d startPose) {
//        return drive.trajectorySequenceBuilder(startPose)
////                .forward(.25)
//                .splineTo(new Vector2d(15, -32), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drop.setPosition(.4))
//                .waitSeconds(1.0)
//                .splineToConstantHeading(new Vector2d(52, -31), Math.toRadians(180))
//                .back(4)
//                .UNSTABLE_addTemporalMarkerOffset(0, this::dropArmPos)
//                .waitSeconds(2.25)
//                .splineToConstantHeading(new Vector2d(0, -10), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(-3, this::resetArmPos)
//                .splineToConstantHeading(new Vector2d(-51, -21), Math.toRadians(180))
//                .waitSeconds(2.0)
//                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> stack.setPosition(1))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(1.0))
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(4, 4, 15.99))
//                .strafeLeft(3.2)//                .waitSeconds(1.0)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> setArmPos(-25, 3000))
//                .waitSeconds(.5)
//                .forward(6)
////                .UNSTABLE_addTemporalMarkerOffset(-.10,()->intake.setPower(0.5))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> setArmPos(150, 500))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> wrist.setPosition(.20))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> intake.setPower(0))
//                .UNSTABLE_addTemporalMarkerOffset(+1.5, () -> guard.setPosition(.7))
//                .waitSeconds(2.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> wrist.setPosition(.46))
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(49, 49, 15.99))
//                .back(7)
//                .splineToConstantHeading(new Vector2d(0, -10), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(+2, () -> setArmPos(500, 3000))
//                .splineToConstantHeading(new Vector2d(54, -35), Math.toRadians(180))
//                .back(5)
//                .UNSTABLE_addTemporalMarkerOffset(+1, this::dropArmPos2)
//                .waitSeconds(3)
//                .build();
//    }

    private TrajectorySequence getRedBoardLeft(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
//                .forward(.25)
                .splineTo(new Vector2d(-38,40),Math.toRadians(0))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(-.2,()->drop.setPosition(.4))
//                .back(2)
                .waitSeconds(1.5)
                .back(6)
                .splineTo(new Vector2d(-42,60),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-.5,()->drop.setPosition(.01))
                .forward(.5)
                .turn(Math.toRadians(-3))
                .waitSeconds(7)
                .strafeLeft(30)
                .forward(5)
                .turn(Math.toRadians(-87))
                .strafeLeft(6.5)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(4, 4, 15.99))
                .back(21)
                .UNSTABLE_addTemporalMarkerOffset(+.5,()->dropArmPos())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.99))
                .waitSeconds(2)
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0,()->resetArmPos())
                .strafeLeft(13)
                .back(8)
                .waitSeconds(20)
                .build();
    }
    private TrajectorySequence getRedBoardCenter(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
//                .forward(.25)
                .splineTo(new Vector2d(-39,34),Math.toRadians(-90))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(-.2,()->drop.setPosition(.4))
//                .back(2)
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(-42,60),Math.toRadians(-90))
                .forward(.5)
                .UNSTABLE_addTemporalMarkerOffset(-.5,()->drop.setPosition(.01))
//                .forward(4)
                .turn(Math.toRadians(-3))
                .waitSeconds(7)
                .strafeLeft(30)
                .forward(5)
                .turn(Math.toRadians(-87))
                .strafeLeft(10.5)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(4, 4, 15.99))
                .back(21)
                .UNSTABLE_addTemporalMarkerOffset(+.5,()->dropArmPos())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.99))
                .waitSeconds(2)
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,()->resetArmPos())
                .strafeLeft(10)
                .back(8)
                .waitSeconds(20)
                .build();
    }

    private TrajectorySequence getRedBoardRight(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-41,45),Math.toRadians(-90))
                .back(2)
                .UNSTABLE_addTemporalMarkerOffset(-.2,()->drop.setPosition(.4))
//                .back(2)
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(-42,60),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-.5,()->drop.setPosition(.01))
                .forward(0.5)
                .turn(Math.toRadians(-3))
                .waitSeconds(7)
                .strafeLeft(30)
                .forward(5)
                .turn(Math.toRadians(-87))
                .strafeLeft(12)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(4, 4, 15.99))
                .back(17)
                .UNSTABLE_addTemporalMarkerOffset(+.5,()->dropArmPos())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.99))
                .waitSeconds(2)
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,()->resetArmPos())
                .strafeLeft(8)
                .back(8)
                .waitSeconds(20)
                .build();
    }

    class samplePipeline extends OpenCvPipeline {
       // Cam2 web = new Cam2();
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
            telemetry.addLine("pipeline running╰(*°▽°*)╯╰(*°▽°*)╯ --> From Camera");
            telemetry.update();
            Rect leftrect = new Rect(30, 165, 10, 10);
            Rect centerrect = new Rect(350, 170, 10, 10);
            Rect rightrect = new Rect(610, 210, 10, 10);

            input.copyTo(output);
            Imgproc.rectangle(output, leftrect, rectColor, 2);
            Imgproc.rectangle(output, centerrect, new Scalar(0,255,0), 2);
            Imgproc.rectangle(output, rightrect, new Scalar(0,0,255), 2);

            leftCrop = YCbCr.submat(leftrect);
            centerCrop = YCbCr.submat(centerrect);
            rightCrop = YCbCr.submat(rightrect);

            Core.extractChannel(leftCrop, leftCrop, 0);
            Core.extractChannel(centerCrop, centerCrop, 0);
            Core.extractChannel(rightCrop, rightCrop, 0);

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


                telemetry.update();
            }

            return (output);


        }

//        @Override
//        public void onViewportTapped() {
//            /*
//             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
//             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
//             * when you need your vision pipeline running, but do not require a live preview on the
//             * robot controller screen. For instance, this could be useful if you wish to see the live
//             * camera preview as you are initializing your robot, but you no longer require the live
//             * preview after you have finished your initialization process; pausing the viewport does
//             * not stop running your pipeline.
//             *
//             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
//             */
//
//            viewportPaused = !viewportPaused;
//
//            if (viewportPaused) {
//                webcam.pauseViewport();
//            } else {
//                webcam.resumeViewport();
//            }
//        }
    }
}
