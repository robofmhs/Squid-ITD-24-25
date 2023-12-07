

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.Service.RobotBaseMovementService;


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
@Autonomous(name="Auto", group="Linear OpMode")

public class Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//     private DcMotorEx flMotor = null;
//     private DcMotorEx frMotor = null;
//     private DcMotorEx blMotor = null;
//     private DcMotorEx brMotor = null;


    @Override
    public void runOpMode() {
        RobotBaseMovementService base = new RobotBaseMovementService();
        Cam2 prop = new Cam2();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int count=0;
         /*
          Initialize the hardware variables. Note that the strings used here as parameters
          to 'get' must correspond to the names assigned during the robot configuration
          step (using the FTC Robot Controller app on the phone).
         */
        base.frMotor = hardwareMap.get(DcMotorEx.class, "fr");
        base.flMotor = hardwareMap.get(DcMotorEx.class, "fl");
        base.blMotor = hardwareMap.get(DcMotorEx.class, "bl");
        base.brMotor = hardwareMap.get(DcMotorEx.class, "br");
        base.flMotor.setDirection(DcMotor.Direction.REVERSE);
        base.blMotor.setDirection(DcMotor.Direction.REVERSE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            while(count==0){
                base.driveFront(500,1000);
                sleep(1500);
                if(prop.Left){base.driveLeft(500,1000);}
                else if(prop.Right){base.driveRight(500,1000);}
                else if(prop.Center){base.driveFront(1500,1000);}
                count+=1;
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
