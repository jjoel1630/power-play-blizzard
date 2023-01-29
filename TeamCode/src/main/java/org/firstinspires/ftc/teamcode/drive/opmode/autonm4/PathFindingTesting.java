package org.firstinspires.ftc.teamcode.drive.opmode.autonm4;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class PathFindingTesting extends LinearOpMode {
    public static Vector2d v1 = new Vector2d(12, -48);
    public static Vector2d v2 = new Vector2d(12, -14);
    public static Vector2d v3 = new Vector2d(24, -10);
    public static Vector2d storage = new Vector2d(68, -12);
    public static Vector2d high = new Vector2d(24, -10);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(60, 36, Math.toRadians(180));
        drive.setPoseEstimate(start);

        Trajectory goToPolePreloaded = drive.trajectoryBuilder(start)
                .lineToConstantHeading(v1)
                .splineTo(v2, Math.toRadians(180))
                .lineToConstantHeading(v3)
                .build();

        Trajectory goToStorage = drive.trajectoryBuilder(goToPolePreloaded.end())
                .splineTo(storage, Math.toRadians(90))
                .build();

        Trajectory goToHigh = drive.trajectoryBuilder(goToStorage.end())
                .splineTo(high, Math.toRadians(180))
                .build();

        waitForStart();

        while(opModeIsActive()) {
//            linearSlide.setTargetPosition(-100);
//            linearSlide.setPower(0.1);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            drive.followTrajectory(goToPolePreloaded);
            drive.followTrajectory(goToStorage);
            drive.followTrajectory(goToHigh);

            break;
        }
    }
}
