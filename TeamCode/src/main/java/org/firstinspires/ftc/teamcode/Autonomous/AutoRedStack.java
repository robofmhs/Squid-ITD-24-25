

 package org.firstinspires.ftc.teamcode.Autonomous;

 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotorEx;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.Autonomous.Service.RobotBaseMovementService;


 @Autonomous(name="AutoRedStack", group="Linear OpMode")

 public class AutoRedStack extends LinearOpMode {

     // Declare OpMode members.
     private ElapsedTime runtime = new ElapsedTime();


     @Override
     public void runOpMode() {
         RobotBaseMovementService base = new RobotBaseMovementService();
         telemetry.addData("Status", "Initialized");
         telemetry.update();
         int count = 0;

         // Initialize the hardware variables. Note that the strings used here as parameters
         // to 'get' must correspond to the names assigned during the robot configuration
         // step (using the FTC Robot Controller app on the phone).
         base.frMotor = hardwareMap.get(DcMotorEx.class, "fr");
         base.flMotor = hardwareMap.get(DcMotorEx.class, "fl");
         base.blMotor = hardwareMap.get(DcMotorEx.class, "bl");
         base.brMotor = hardwareMap.get(DcMotorEx.class, "br");
         base.flMotor.setDirection(DcMotorEx.Direction.REVERSE);
         base.blMotor.setDirection(DcMotorEx.Direction.REVERSE);
         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
         // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

         // Wait for the game to start (driver presses PLAY)
         waitForStart();
         runtime.reset();

         // run until he end of the match (driver presses STOP)
         while (opModeIsActive()) {
             while(count==0){
             base.driveFront(150,2000);
             sleep(1500);
             base.driveRight(2520,2000);
             count+=1;
             }

             // Show the elapsed game time and wheel power.
             telemetry.addData("Status", "Run Time: " + runtime.toString());
             // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
             telemetry.update();
         }
     }
 }
