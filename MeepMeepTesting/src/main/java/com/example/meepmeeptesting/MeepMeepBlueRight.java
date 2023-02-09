package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Score Preloaded;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
                .setDimensions(14.5, 14.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32.81, 64.87, Math.toRadians(270.00)))
                                .splineToConstantHeading(new Vector2d(12.62, 53.59), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(12.62, 17.52), Math.toRadians(270))
                                .splineTo(new Vector2d(19.00, 4.01), Math.toRadians(315))
                                .build());

//        // Go to storage 1
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
                .setDimensions(14.5, 14.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(19.00, 4.01, Math.toRadians(315)))
                                .lineToConstantHeading(new Vector2d(12.17, 20.49))
                                .build());

        // Go to storage 2
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
                .setDimensions(14.5, 14.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.17, 20.49, Math.toRadians(315)))
                                .splineTo(new Vector2d(63.67, 11.75), Math.toRadians(0))
                                .build());

        // Score storage cone in one try v1
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
                .setDimensions(14.5, 14.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(63.97, 11.75, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(43.36, 13.69, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(28.56, 5.21, Math.toRadians(225)))
                                .build());

        // Go to storage cone in one try vfinal
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
                .setDimensions(14.5, 14.0)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(28.56, 5.21, Math.toRadians(225)))
                                .lineToLinearHeading(new Pose2d(43.36, 13.69, Math.toRadians(0.00)))
                                .lineToLinearHeading(new Pose2d(63.97, 11.75, Math.toRadians(0.00)))
                                .build());


        // Score storage cone 1
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(63.84, 12.62, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(8.31, 11.88, Math.toRadians(0)))
//                                .build());
//
//        // Score storage cone 2
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(8.31, 11.88, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(19.60, 4.60, Math.toRadians(315)))
//                                .build());
////        // Position park 1
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(19.60, 4.60, Math.toRadians(315)))
//                                .lineToLinearHeading(new Pose2d(11.58, 15.44, Math.toRadians(270.00)))
//                                .build());
////
////        // Position park 2
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(11.58, 15.44, Math.toRadians(270.00)))
//                                .lineToLinearHeading(new Pose2d(12.62, 35.93, Math.toRadians(180)))
//                                .build());
//
////         Park for index 0
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.62, 35.93, Math.toRadians(180)))
//                                .turn(Math.toRadians(-90))
//                                .build());
//
//        // Park for index 1
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.62, 35.93, Math.toRadians(180)))
//                                .lineToConstantHeading(new Vector2d(36.22, 35.63))
//                                .build());
//
//        // Park for index 0
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.62, 35.93, Math.toRadians(180)))
//                                .lineToConstantHeading(new Vector2d(59.68, 35.33))
//                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}