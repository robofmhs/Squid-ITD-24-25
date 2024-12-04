package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controls.DriveControl;
import org.firstinspires.ftc.teamcode.Controls.IntakeControl;
import org.firstinspires.ftc.teamcode.Controls.OuttakeControl;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeDiffy;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeGripper;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeGripper;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeWrist;
import org.firstinspires.ftc.teamcode.Subsystems.PivotArm;
import org.firstinspires.ftc.teamcode.Subsystems.PivotSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@TeleOp
@Config
public class ImprovedRC_Base extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DriveControl driveControl = new DriveControl(robot.drivebase);
        IntakeControl intakeControl = new IntakeControl(robot.IntakeSlides, robot.IntakeDiffy, robot.IntakeGripper);
        OuttakeControl outtakeControl = new OuttakeControl(robot.wrist, robot.OuttakeGripper, robot.arm, robot.pivotSlides);
        Subsystem[] subsystems = new Subsystem[]{robot};

        waitForStart();

        while(opModeIsActive()) {
            for(Subsystem system : subsystems) system.update();
            telemetry.update();
            driveControl.update(gamepad1, gamepad2);
            intakeControl.update(gamepad1, gamepad2);
            outtakeControl.update(gamepad1, gamepad2);
        }
    }
}
