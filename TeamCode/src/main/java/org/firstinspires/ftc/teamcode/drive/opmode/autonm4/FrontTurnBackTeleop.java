package org.firstinspires.ftc.teamcode.drive.opmode.autonm4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name="front turn back")
public class FrontTurnBackTeleop extends LinearOpMode {
    public static Vector2d v1 = new Vector2d(12, -48);
    public static Vector2d v2 = new Vector2d(12, -14);
    public static Vector2d v3 = new Vector2d(24, -10);
    public static Vector2d storage = new Vector2d(68, -12);
    public static Vector2d high = new Vector2d(24, -10);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(start);

        Trajectory t1 = drive.trajectoryBuilder(start, false)
                .back(50)
                .build();

// We just add a 90 degree heading rotation to traj1.end()
        Trajectory t2 = drive.trajectoryBuilder(t1.end().plus(new Pose2d(0, 0, Math.toRadians(-45))), false)
                .forward(15)
                .build();

        Trajectory t3 = drive.trajectoryBuilder(t2.end(), false)
                .back(15)
                .build();

        Trajectory t4 = drive.trajectoryBuilder(t3.end().plus(new Pose2d(0, 0, Math.toRadians(0))), false)
                .forward(50)
                .build();

        waitForStart();

        while(opModeIsActive()) {
//            linearSlide.setTargetPosition(-100);
//            linearSlide.setPower(0.1);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            for(int i = 0; i < 5; ++i) {
                drive.followTrajectory(t1);
                drive.turn(Math.toRadians(-45));
                drive.followTrajectory(t2);
                drive.followTrajectory(t3);
                drive.turn(Math.toRadians(45));
                drive.followTrajectory(t4);
            }

            break;
        }
    }
}
