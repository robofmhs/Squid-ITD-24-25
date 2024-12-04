package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot implements Subsystem {
    public PivotArm arm;
    public PivotSlides pivotSlides;
    public OuttakeWrist wrist;
    public OuttakeGripper OuttakeGripper;
    public Drivebase drivebase;
    public IntakeGripper IntakeGripper;
    public IntakeDiffy IntakeDiffy;
    public IntakeSlides IntakeSlides;

    public Robot(HardwareMap map) {
        arm = new PivotArm(map);
        pivotSlides = new PivotSlides(map);
        wrist = new OuttakeWrist(map);
        OuttakeGripper = new OuttakeGripper(map);
        drivebase = new Drivebase(map);
        IntakeGripper = new IntakeGripper(map);
        IntakeDiffy = new IntakeDiffy(map);
        IntakeSlides = new IntakeSlides(map);
    }
    @Override
    public void update() {
        arm.update();
        pivotSlides.update();
        wrist.update();
        OuttakeGripper.update();
        drivebase.update();
        IntakeGripper.update();
        IntakeDiffy.update();
        IntakeSlides.update();
    }
}
