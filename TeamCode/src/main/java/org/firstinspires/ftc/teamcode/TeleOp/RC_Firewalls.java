
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "RC_FirewallsAnd", group = "Linear Opmode")
public class RC_Firewalls extends LinearOpMode {



    // NEVER CHANGE SPEED
    private static final float sensitivity = .6f;


    enum StrafeDirection {
        BACKWARD(0.8f * sensitivity, -0.8f * sensitivity, 0.8f * sensitivity, -0.8f * sensitivity),
        FORWARD(-0.8f * sensitivity, 0.8f * sensitivity, -0.8f * sensitivity, 0.8f * sensitivity),
        LEFT(.6f, .6f, -.6f, -.6f),
        RIGHT(-.6f, -.6f, .6f, .6f),
        START(0.0f, 0.0f, 0.0f, 0.0f),
        TURNLEFT(0.5f, 0.5f, 0.5f, 0.5f),
        TURNRIGHT(-0.5f, -0.5f, -0.5f, -0.5f);


        private final float flMotorPower;
        private final float frMotorPower;
        private final float blMotorPower;
        private final float brMotorPower;


        StrafeDirection(float flMotorPower, float frMotorPower, float blMotorPower, float brMotorPower) {
            this.flMotorPower = flMotorPower;
            this.frMotorPower = frMotorPower;
            this.blMotorPower = blMotorPower;
            this.brMotorPower = brMotorPower;


        }
    }


    double flPower;
    double frPower;
    double blPower;
    double brPower;


    void setPWR(StrafeDirection Dir) {
        flPower = Dir.flMotorPower;
        frPower = Dir.frMotorPower;
        blPower = Dir.blMotorPower;
        brPower = Dir.brMotorPower;
    }




    private DcMotorEx flMotor = null;
    private DcMotorEx frMotor = null;
    private DcMotorEx blMotor = null;
    private DcMotorEx brMotor = null;
    private DcMotorEx arm = null;
    private DcMotorEx fly = null;
    private DcMotorEx intake = null;
    private DcMotorEx lift = null;
    private CRServo wrist = null;
    private CRServo wrist2 = null;

    private CRServo servoo = null;





    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();



    private void updateMotorPower() {
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);
    }

    void strafe(StrafeDirection Dir) {
        flPower = Dir.flMotorPower;
        frPower = Dir.frMotorPower;
        blPower = Dir.blMotorPower;
        brPower = Dir.brMotorPower;
        updateMotorPower();
    }
    public final void brake() {
        flPower = 0.0;
        frPower = 0.0;
        blPower = 0.0;
        brPower = 0.0;
        updateMotorPower();
    }
    void reduceSpeed() {
        flPower = flPower*0.3;
        frPower = frPower*0.3;
        blPower = blPower*0.3;
        brPower = brPower*0.3;
        updateMotorPower();
    }
    void increaseSpeed() {
        flPower = flPower*3.0;
        frPower = frPower*3.0;
        blPower = blPower*3.0;
        brPower = brPower*3.0;
        updateMotorPower();
    }



//    void	closeGripper()	{
////	gripper
//        if	(wrist2.getPosition()	>=	0.7	&&	wrist2.getPosition()	<	0.71)	{
//            wrist2.setPosition(.1);
//
//        }
//        else	if	(wrist2.getPosition()	>=	.1)	{
//            wrist2.setPosition(0.7);
//
//        }
//        else	{
//            wrist2.setPosition(0.1);
//        }
//
//    }

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        waitForStart();
        runtime.reset();

        frMotor = hardwareMap.get(DcMotorEx.class, "fr");
        flMotor = hardwareMap.get(DcMotorEx.class, "fl");
        blMotor = hardwareMap.get(DcMotorEx.class, "bl");
        brMotor = hardwareMap.get(DcMotorEx.class, "br");
        setPWR(StrafeDirection.START);
        arm  = hardwareMap.get(DcMotorEx.class, "arm");
        fly  = hardwareMap.get(DcMotorEx.class, "fly");
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        lift  = hardwareMap.get(DcMotorEx.class, "lift");
        wrist   = hardwareMap.get(CRServo.class, "wrist");
        wrist2   = hardwareMap.get(CRServo.class, "wrist2");
        servoo   = hardwareMap.get(CRServo.class, "servoo");
        double    SlidePositionPositive;
        double    SlidePositionNegative;
//        double    SlidePosition    =    wrist.getPosition();
        double    SlidePN;
        double    SlideMin    =    0.1;
        double    SlideMax    =    1.0;
        double    SlideIncrement    =    0.005;

        double    SlidePositionPositive2;
        double    SlidePositionNegative2;
//        double    SlidePosition2    =    wrist2.getPosition();
        double    SlidePN2;
        double    SlideMin2    =    0.1;
        double    SlideMax2   =    1.0;
        double    SlideIncrement2    =    0.005;
// slide
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.left_stick_x < 0 || gamepad1.left_stick_x > 0 ||
                    gamepad1.left_stick_y < 0 || gamepad1.left_stick_y > 0 ||
                    gamepad1.right_stick_x < 0 || gamepad1.right_stick_x > 0) {

// turret
                telemetry.addData("working", "yes");
                // if (turret.getCurrentPosition()>= 650){
                //         turret.setPower(0.0);
                // }
                // else if(turret.getCurrentPosition()<=-720){
                //         turret.setPower(0.0);
                // }

                // base

                if (gamepad1.left_stick_x < 0) {
                    strafe(StrafeDirection.LEFT);
                }

                if (gamepad1.left_stick_x > 0) {
                    strafe(StrafeDirection.RIGHT);
                }

                if (gamepad1.left_stick_y > 0) {
                    strafe(StrafeDirection.BACKWARD);
                }

                if (gamepad1.left_stick_y < 0) {
                    strafe(StrafeDirection.FORWARD);
                }

                if (gamepad1.right_stick_x < 0) {
                    strafe(StrafeDirection.TURNLEFT);
                }

                if (gamepad1.right_stick_x > 0) {
                    strafe(StrafeDirection.TURNRIGHT);
                }

            }
            else {
                brake();
            }

// gripper
            if (gamepad1.left_bumper) {
                reduceSpeed();
            }

            if (gamepad1.right_bumper) {
                increaseSpeed();
            }
            if (gamepad2.a){
                arm.setVelocity(4000);
                arm.setTargetPosition(arm.getCurrentPosition()+ 100);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                // arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }
            else if (gamepad2.b){
                arm.setVelocity(4000);
                arm.setTargetPosition(arm.getCurrentPosition()- 100);
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                // arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            }
            else {
                arm.setVelocity(0);
                arm.setTargetPosition(arm.getCurrentPosition());
                arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            }
//            if	(gamepad2.right_bumper)	{
//                closeGripper();
//                while	(gamepad2.right_bumper){
//
//                }
//            }
            //lift
            if (gamepad2.right_bumper){


                lift.setPower(.3);
            }
            // arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            else if (gamepad2.left_bumper){

                lift.setPower(-.3);
                // arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            }
            else{
                lift.setPower(0);
            }
//            if(gamepad2.x){
//                lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            }\6y6
////flywheeel

            if (gamepad2.y&& gamepad1.y){
                fly.setPower(1.0);
            }
            else{
                fly.setPower(0);
            }

            if (gamepad2.right_trigger>0){
                intake.setPower(-1.0*gamepad2.right_trigger);
            }
            else if(gamepad2.left_trigger>0){
                intake.setPower(1.0*gamepad2.left_trigger);
            }
            else{
                intake.setPower(0);
            }
            if (gamepad2.left_stick_y>0){
                wrist.setPower(.17);
            } else if (gamepad2.left_stick_y<0) {
                wrist.setPower(-.17);
            }
            else {
                wrist.setPower(0);
            }
            if (gamepad2.right_stick_y>0){
                wrist2.setPower(.2);
            } else if (gamepad2.right_stick_y<0) {
                wrist2.setPower(-.2);
            }
            else {
                wrist2.setPower(0);
            }

//            SlidePosition    =    wrist.getPosition();

//            if    (gamepad2.left_stick_y>0){
//                SlidePositionNegative    =    Range.clip(SlidePosition    -    SlideIncrement,    SlideMin,    SlideMax);
//                wrist.setPosition(SlidePositionNegative);
//            }
//            else if    (gamepad2.left_stick_y<0){
//                SlidePositionPositive    =    Range.clip(SlidePosition    +    SlideIncrement,    SlideMin,    SlideMax);
//                wrist.setPosition(SlidePositionPositive);
//            }
//            else {
//                SlidePosition    =    wrist.getPosition();
//                wrist.setPosition(SlidePosition);
//            }
//            SlidePosition2    =    wrist2.getPosition();
//            if    (gamepad2.right_stick_y>0){
//                SlidePositionNegative2    =    Range.clip(SlidePosition2    -    SlideIncrement2,    SlideMin2,    SlideMax2);
//                wrist2.setPosition(SlidePositionNegative2);
//            }
//            else if    (gamepad2.right_stick_y<0){
//                SlidePositionPositive2    =    Range.clip(SlidePosition2    +    SlideIncrement2,    SlideMin2,    SlideMax2);
//                wrist2.setPosition(SlidePositionPositive2);
//            }
//            else {
//                SlidePosition2    =    wrist2.getPosition();
//                wrist2.setPosition(SlidePosition2);
//            }








            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("frontLeft", flMotor.getCurrentPosition());
            telemetry.addData("bl", blMotor.getCurrentPosition());
            telemetry.addData("frontRight", frMotor.getCurrentPosition());
            telemetry.addData("br", brMotor.getCurrentPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("intake", intake.getCurrentPosition());
            telemetry.addData("lift", lift.getCurrentPosition());
            // telemetry.addData("hi: ",replugController);
            telemetry.update();
        }
    }
}


