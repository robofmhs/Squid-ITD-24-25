package org.firstinspires.ftc.teamcode.Controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;

public class DriveControl {
    public Drivebase drivebase;

    public DriveControl(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    public void update(Gamepad g1, Gamepad g2) {
        drivebase.setPower(-g1.left_stick_y, -g1.left_stick_x, -g1.right_stick_x);
        drivebase.update();
    }
}