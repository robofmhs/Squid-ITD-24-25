package org.firstinspires.ftc.teamcode.ROADRUNNER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="TestOne", group="Linear OpMode")

public class Paths extends LinearOpMode {
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
//        Trajectory myTrajectory =drive.trajectoryBuilder(new Pose2d())
//                .splineToConstantHeading(new Vector2d(10,20),Math.toRadians(-3))
//                .build();
//        Trajectory myTrajectory2 =drive.trajectoryBuilder(myTrajectory.end())
//                .splineToConstantHeading(new Vector2d(-10,20),Math.toRadians(-3))
//                .build();
        Pose2d startPose = new Pose2d(-36, -(70-9), Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        waitForStart();
        if (isStopRequested()) {
            return;
        }



//        TrajectorySequence RedBoardRight= drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(37, -31), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drop.setPosition(.4))
//                .waitSeconds(1.0)
//                .splineToConstantHeading(new Vector2d(54, -35), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> dropArmPos())
//                .waitSeconds(2.25)
//                .splineToConstantHeading(new Vector2d(0, -10), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> resetArmPos())
//                .splineToConstantHeading(new Vector2d(-51, -21), Math.toRadians(180))
//                .waitSeconds(2.0)
//                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> stack.setPosition(1))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(1.0))
//                .setVelConstraint(drive.getVelocityConstraint(4, 4, 15.99))
//                .strafeLeft(3.4)//                .waitSeconds(1.0)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> setArmPos(9, 3000))
//                .waitSeconds(.5)
//                .forward(6)
////                .UNSTABLE_addTemporalMarkerOffset(-.10,()->intake.setPower(0.5))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> setArmPos(150, 500))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> wrist.setPosition(.20))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> intake.setPower(0))
//                .UNSTABLE_addTemporalMarkerOffset(+1.5, () -> guard.setPosition(.7))
//                .waitSeconds(2.5)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> wrist.setPosition(.46))
//                .setVelConstraint(drive.getVelocityConstraint(49, 49, 15.99))
//                .back(7)
//                .splineToConstantHeading(new Vector2d(0, -15), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(-.2, () -> setArmPos(500, 3000))
//                .splineToConstantHeading(new Vector2d(54 - .5, -32), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> dropArmPos2())
//                .waitSeconds(3)
//                .build();


        TrajectorySequence RedBoardCenter = drive.trajectorySequenceBuilder(startPose)
//                .forward(.25)
                .splineTo(new Vector2d(-38,-28),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.2,()->drop.setPosition(.5))
//                .back(1)
//                .back(2)
                .waitSeconds(1.5)
                .back(3)
                .splineTo(new Vector2d(-40,-60),Math.toRadians(-90))
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(-.5,()->drop.setPosition(.01))
//                .forward(4)
                .turn(Math.toRadians(-3))
                .strafeRight(30)
                .forward(5)
                .turn(Math.toRadians(93))
                .strafeRight(5)
                .back(11)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(4, 4, 15.99))
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(+.5,()->dropArmPos())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, 40, 15.99))
                .waitSeconds(2)
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->resetArmPos())
                .strafeLeft(8)
                .back(8)
                .waitSeconds(20)
                .build();
//        TrajectorySequence RedBoardLeft = drive.trajectorySequenceBuilder(startPose)
////                .forward(.25)
//                .splineTo(new Vector2d(15, -32), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> drop.setPosition(.4))
//                .waitSeconds(1.0)
//                .splineToConstantHeading(new Vector2d(52, -30), Math.toRadians(180))
//                .back(4)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> dropArmPos())
//                .waitSeconds(2.25)
//                .splineToConstantHeading(new Vector2d(0, -10), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(-3, () -> resetArmPos())
//                .splineToConstantHeading(new Vector2d(-51, -21), Math.toRadians(180))
//                .waitSeconds(2.0)
//                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> stack.setPosition(1))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.setPower(1.0))
//                .setVelConstraint(drive.getVelocityConstraint(4, 4, 15.99))
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
//                .setVelConstraint(drive.getVelocityConstraint(49, 49, 15.99))
//                .back(7)
//                .splineToConstantHeading(new Vector2d(0, -10), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(+2, () -> setArmPos(500, 3000))
//                .splineToConstantHeading(new Vector2d(54, -35), Math.toRadians(180))
//                .back(5)
//                .UNSTABLE_addTemporalMarkerOffset(+1, () -> dropArmPos2())
//                .waitSeconds(3)
//                .build();

        drive.followTrajectorySequence(RedBoardCenter);
    }
}
