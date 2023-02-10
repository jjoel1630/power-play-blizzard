package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Score Preloaded;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
                .setDimensions(14.5, 14.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, -64.87, Math.toRadians(90.00)))
                                .splineToConstantHeading(new Vector2d(-12.62, -53.59), Math.toRadians(90.00))
                                .splineTo(new Vector2d(-12.62, -17.52), Math.toRadians(90.00))
                                .splineTo(new Vector2d(-19.00, -5.5), Math.toRadians(135))
                                .build());


//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-19.89, 4.45, Math.toRadians(135.00)))
//                                .lineToConstantHeading(new Vector2d(-11.88, 11.28))
//                                .build());
////
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-11.88, -11.28, Math.toRadians(135)))
//                                .lineToLinearHeading(new Pose2d(-12.92, -36.15, Math.toRadians(90.00)))
//                                .turn(Math.toRadians(-90))
//                                .build());
//
//
//////        // Park for index 0
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-12.92, -36.15, Math.toRadians(0)))
//                                .turn(Math.toRadians(-90))
//                                .build());
////////
////////        // Park for index 1
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-12.92, -36.15, Math.toRadians(0)))
//                                .lineToConstantHeading(new Vector2d(-36.22, -36.15))
//                                .build());
//////
//////        // Park for index 2
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-12.92, -36.15, Math.toRadians(0)))
//                                .lineToConstantHeading(new Vector2d(-59.68, -36.15))
//                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}