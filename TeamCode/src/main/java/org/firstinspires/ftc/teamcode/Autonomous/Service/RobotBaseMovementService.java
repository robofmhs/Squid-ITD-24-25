

 package org.firstinspires.ftc.teamcode.Autonomous.Service;

 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;


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


 public class RobotBaseMovementService  {

     // Declare OpMode members.

     public DcMotorEx flMotor = null;
     public DcMotorEx frMotor = null;
     public DcMotorEx blMotor = null;
     public DcMotorEx brMotor = null;
     public void driveFront(int amount , int speed) {

         flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         flMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         frMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         blMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         brMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         // turret.setTargetPosition(5);

         flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         flMotor.setVelocity(speed);
         frMotor.setVelocity(speed);
         blMotor.setVelocity(speed);
         brMotor.setVelocity(speed);


     }
     public void driveBack(int amount, int speed) {

         flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         flMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         frMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         blMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         brMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         // turret.setTargetPosition(5);

         flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         flMotor.setVelocity(speed);
         frMotor.setVelocity(speed);
         blMotor.setVelocity(speed);
         brMotor.setVelocity(speed);


     }
     public void driveLeft(int amount, int speed) {

         flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         flMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         frMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         blMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         brMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         // turret.setTargetPosition(5);

         flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         flMotor.setVelocity(speed);
         frMotor.setVelocity(speed);
         blMotor.setVelocity(speed);
         brMotor.setVelocity(speed);


     }
     public void driveRight(int amount, int speed) {
         flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


         flMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         frMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         blMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         brMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         // turret.setTargetPosition(5);

         flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         flMotor.setVelocity(speed);
         frMotor.setVelocity(speed);
         blMotor.setVelocity(speed);
         brMotor.setVelocity(speed);


     }
     public void turnRight(int amount, int speed) {
         flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


         flMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         frMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         blMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         brMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         // turret.setTargetPosition(5);

         flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         flMotor.setVelocity(speed);
         frMotor.setVelocity(speed);
         blMotor.setVelocity(speed);
         brMotor.setVelocity(speed);


     }
     public void turnLeft(int amount, int speed) {
         flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


         flMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         frMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         blMotor.setTargetPosition(flMotor.getCurrentPosition()-amount);
         brMotor.setTargetPosition(flMotor.getCurrentPosition()+amount);
         // turret.setTargetPosition(5);

         flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         flMotor.setVelocity(speed);
         frMotor.setVelocity(speed);
         blMotor.setVelocity(speed);
         brMotor.setVelocity(speed);


     }

    /* @Override
     public void runOpMode() {
         AprilTag aprilTag = new AprilTag();
         aprilTag.telemetryAprilTag();
         telemetry.addData("Status", "Initialized");
         telemetry.update();
         int count=0;
         // Initialize the hardware variables. Note that the strings used here as parameters
         // to 'get' must correspond to the names assigned during the robot configuration
         // step (using the FTC Robot Controller app on the phone).
         frMotor = hardwareMap.get(DcMotorEx.class, "fr");
         flMotor = hardwareMap.get(DcMotorEx.class, "fl");
         blMotor = hardwareMap.get(DcMotorEx.class, "bl");
         brMotor = hardwareMap.get(DcMotorEx.class, "br");
             flMotor.setDirection(DcMotor.Direction.REVERSE);
             blMotor.setDirection(DcMotor.Direction.REVERSE);


         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
         // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

         // Wait for the game to start (driver presses PLAY)
         waitForStart();
         runtime.reset();

         // run until the end of the match (driver presses STOP)
         while (opModeIsActive()) {
             while(count==0){

             driveLeft(2020,2000);
             count+=1;
             }




             // Show the elapsed game time and wheel power.
             telemetry.addData("Status", "Run Time: " + runtime.toString());
             // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
             telemetry.update();
         }
     }*/
 }
