package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Right {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Score Preloaded;
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 4.9, Math.toRadians(184.02607784577722), 12.23)
//                .setDimensions(14.0, 14.5)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(31.50, -64.75, Math.toRadians(90.00)))
//                                .lineToConstantHeading(new Vector2d(31.50, -60.00))
//                                .lineToConstantHeading(new Vector2d(12.00, -60.00))
//                                .lineToConstantHeading(new Vector2d(12.00, -11.00))
//                                .lineToConstantHeading(new Vector2d(24.00, -11.00))
//                                .lineToConstantHeading(new Vector2d(24.00, -6.00))
//                                .build());

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 24, 4.9, Math.toRadians(184.02607784577722), 12.23)
                .setDimensions(14.0, 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(31.50, -64.75, Math.toRadians(90.00)))
                                .lineToConstantHeading(new Vector2d(12.00, -55.00))
                                .lineToConstantHeading(new Vector2d(12.00, -11.00))
                                .lineToConstantHeading(new Vector2d(24.00, -11.00))
                                .lineToConstantHeading(new Vector2d(24.00, -6.00))
                                .build());

        // move back from pole
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(20.93, -3.56, Math.toRadians(45.00)))
//                                .lineToConstantHeading(new Vector2d(12.47, -14.25))
//                                .build());

        // get ready to park
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.47, -14.25, Math.toRadians(45.00)))
//                                .lineToLinearHeading(new Pose2d(12.47, -35.04, Math.toRadians(90.00)))
//                                .build());

        // park index 0
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.47, -35.04, Math.toRadians(90.00)))
//                                .turn(Math.toRadians(180))
//                                .build());

        // park index 1
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.47, -35.04, Math.toRadians(90.00)))
//                                .turn(Math.toRadians(90))
//                                .lineToConstantHeading(new Vector2d(36.07, -35.04))
//                                .build());

        // park index 2
//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(24, 24, 3.71, Math.toRadians(184.02607784577722), 11.2)
//                .setDimensions(14.5, 14.0)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(12.47, -35.04, Math.toRadians(90.00)))
//                                .turn(Math.toRadians(90))
//                                .lineToConstantHeading(new Vector2d(58.64, -35.04))
//                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}