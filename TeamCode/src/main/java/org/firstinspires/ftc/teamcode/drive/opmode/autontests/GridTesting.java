package org.firstinspires.ftc.teamcode.drive.opmode.autontests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name="Grid Testing", group="Neat Unit Tests")
public class GridTesting extends LinearOpMode {
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(60, 36, Math.toRadians(180)));

        Trajectory goToStorage = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(225)))
                .build();

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            /* --------------- GO TO POLE --------------- */
            drive.followTrajectory(goToStorage);

            while(drive.isBusy()) {}
        }
    }
}
