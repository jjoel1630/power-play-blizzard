package org.firstinspires.ftc.teamcode.drive.opmode.autonm4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name="coord test")
public class CoordTest extends LinearOpMode {
    public static int x1 = 10;
    public static int y1 = 0;
    public static int h1 = 0;

    public static int x2 = 0;
    public static int y2 = 0;
    public static int h2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(start);

        Trajectory test1 = drive.trajectoryBuilder(start)
                .splineToConstantHeading(new Vector2d(x1, y1), Math.toRadians(h1))
                .build();

        Trajectory test2 = drive.trajectoryBuilder(test1.end(), true)
                .splineToConstantHeading(new Vector2d(x2, y2), Math.toRadians(h2))
                .build();

        waitForStart();

        while(opModeIsActive()) {
            drive.followTrajectory(test1);
            drive.followTrajectory(test2);

            break;
        }
    }
}
