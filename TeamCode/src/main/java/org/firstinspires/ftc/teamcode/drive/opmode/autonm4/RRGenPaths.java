package org.firstinspires.ftc.teamcode.drive.opmode.autonm4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="Meep Meep Paths")
public class RRGenPaths extends LinearOpMode {
    public static int POSITION = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d start = new Pose2d(-32.81, 66.25, Math.toRadians(270.00));

        drive.setPoseEstimate(start);

        TrajectorySequence scorePreloaded = drive.trajectorySequenceBuilder(new Pose2d(-32.81, 66.25, Math.toRadians(270.00)))
            .splineToConstantHeading(new Vector2d(-12.32, 49.88), Math.toRadians(-89.36))
            .splineTo(new Vector2d(-20.19, 5.34), Math.toRadians(225.00))
            .build();

        TrajectorySequence goToStorage = drive.trajectorySequenceBuilder(scorePreloaded.end())
                .lineToLinearHeading(new Pose2d(-13.06, 12.32, Math.toRadians(180.00)))
                .splineTo(new Vector2d(-62.80, 12.32), Math.toRadians(180.00))
                .build();

        TrajectorySequence scoreStorage1 = drive.trajectorySequenceBuilder(goToStorage.end())
                .lineToLinearHeading(new Pose2d(-13.06, 12.32, Math.toRadians(180.00)))
                .build();

        TrajectorySequence scoreStorage2 = drive.trajectorySequenceBuilder(scoreStorage1.end())
                .lineToLinearHeading(new Pose2d(-20.19, 5.34, Math.toRadians(225.00)))
                .build();

        TrajectorySequence parkPlacement =  drive.trajectorySequenceBuilder(scoreStorage2.end())
                .lineToLinearHeading(new Pose2d(-12.02, 16.92, Math.toRadians(225.00)))
                .lineToLinearHeading(new Pose2d(-12.17, 36.82, Math.toRadians(0.00)))
                .build();

        TrajectorySequence parkSquare1 =  drive.trajectorySequenceBuilder(parkPlacement.end())
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence parkSquare3 = drive.trajectorySequenceBuilder(parkPlacement.end())
                .lineToConstantHeading(new Vector2d(-61.61, 33.33))
                .build();

        TrajectorySequence parkSquare2 = drive.trajectorySequenceBuilder(parkPlacement.end())
                .lineToConstantHeading(new Vector2d(-36.37, 34.89))
                .build();

        waitForStart();

        while(opModeIsActive()) {
            drive.followTrajectorySequence(scorePreloaded);
            drive.followTrajectorySequence(goToStorage);
            drive.followTrajectorySequence(scoreStorage1);
            drive.followTrajectorySequence(scoreStorage2);
            drive.followTrajectorySequence(parkPlacement);

            if(POSITION == 0) {
                drive.followTrajectorySequence(parkSquare1);
            } else if(POSITION == 1) {
                drive.followTrajectorySequence(parkSquare2);
            }   else if(POSITION == 2) {
                drive.followTrajectorySequence(parkSquare3);
            }

            break;
        }
    }
}
