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
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d start = new Pose2d(-34.81, 63.47, Math.toRadians(270.0));

        drive.setPoseEstimate(start);

        TrajectorySequence scorePreloaded = drive.trajectorySequenceBuilder(new Pose2d(-34.81, 63.47, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-11.75, 50.72), Math.toRadians(-88.06))
                .splineTo(new Vector2d(-21.75, 2.05), Math.toRadians(239.74))
                .build();

        waitForStart();

        while(opModeIsActive()) {
            drive.followTrajectorySequence(scorePreloaded);

            break;
        }
    }
}
