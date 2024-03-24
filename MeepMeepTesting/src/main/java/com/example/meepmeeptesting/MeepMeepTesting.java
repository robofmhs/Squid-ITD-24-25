package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d startPose = new Pose2d(-36, (70-9), Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49, 49, Math.toRadians(144), Math.toRadians(144), 15.99)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(-41,44),Math.toRadians(-90))
                                .waitSeconds(.8)
                                .splineToConstantHeading(new Vector2d(-36,(70-11)),Math.toRadians(-90))
                                .strafeLeft(50)
                                .turn(Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(50, 28), Math.toRadians(180))
                                .back(5)
                                .waitSeconds(3)
                                .forward(5)
                                .strafeLeft(18)
                                .back(8)



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}