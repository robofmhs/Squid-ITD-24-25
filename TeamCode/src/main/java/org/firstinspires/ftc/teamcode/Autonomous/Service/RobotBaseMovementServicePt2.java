

 package org.firstinspires.ftc.teamcode.Autonomous.Service;

 import static java.lang.Thread.sleep;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.teamcode.Autonomous.RobotBaseMovementService;
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


 public class RobotBaseMovementServicePt2 {

     // Declare OpMode members.

     public DcMotorEx flMotor = null;
     public DcMotorEx frMotor = null;
     public DcMotorEx blMotor = null;
     public DcMotorEx brMotor = null;
     public DcMotorEx arm =null;
     public Servo wrist =null;
     public Servo guard =null;
     
     public double buffer = 1;
     public void driveFront(long  amount , double speed) throws InterruptedException {

         flMotor.setPower(speed*buffer);
         frMotor.setPower(speed*buffer);
         blMotor.setPower(speed*buffer);
         brMotor.setPower(speed*buffer);
         sleep(amount);
         flMotor.setPower(0);
         frMotor.setPower(0);
         blMotor.setPower(0);
         brMotor.setPower(0);


     }
     public void driveBack(long amount, double speed) throws InterruptedException {

         flMotor.setPower(-speed*buffer);
         frMotor.setPower(-speed*buffer);
         blMotor.setPower(-speed*buffer);
         brMotor.setPower(-speed*buffer);
         sleep(amount);
         flMotor.setPower(0);
         frMotor.setPower(0);
         blMotor.setPower(0);
         brMotor.setPower(0);


     }
     public void driveLeft(long amount, double speed) throws InterruptedException {

         flMotor.setPower(-speed*buffer);
         frMotor.setPower(speed*buffer);
         blMotor.setPower(speed*buffer);
         brMotor.setPower(-speed*buffer);
         sleep(amount);
         flMotor.setPower(0);
         frMotor.setPower(0);
         blMotor.setPower(0);
         brMotor.setPower(0);


     }
     public void driveRight(long amount, double speed) throws InterruptedException {
         flMotor.setPower(speed*buffer);
         frMotor.setPower(-speed*buffer);
         blMotor.setPower(-speed*buffer);
         brMotor.setPower(speed*buffer);
         sleep(amount);
         flMotor.setPower(0);
         frMotor.setPower(0);
         blMotor.setPower(0);
         brMotor.setPower(0);


     }
     public void turnRight(int amount, double speed) throws InterruptedException {
         flMotor.setPower(speed*buffer);
         frMotor.setPower(-speed*buffer);
         blMotor.setPower(speed*buffer);
         brMotor.setPower(-speed*buffer);
         sleep(amount);
         flMotor.setPower(0);
         frMotor.setPower(0);
         blMotor.setPower(0);
         brMotor.setPower(0);
     }
     public void turnLeft(int amount, double speed) throws InterruptedException {
         flMotor.setPower(-speed*buffer);
         frMotor.setPower(speed*buffer);
         blMotor.setPower(-speed*buffer);
         brMotor.setPower(speed*buffer);
         sleep(amount);
         flMotor.setPower(0);
         frMotor.setPower(0);
         blMotor.setPower(0);
         brMotor.setPower(0);
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
