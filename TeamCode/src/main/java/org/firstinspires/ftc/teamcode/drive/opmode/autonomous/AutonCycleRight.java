package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Disabled
@Config
@Autonomous(name="Auton Cycle Right", group="Goated Auton")
public class AutonCycleRight extends LinearOpMode {
    /* --------------- GLOBAL VARS --------------- */
    PowerPlayBot robot = new PowerPlayBot();
    ElapsedTime timer;

    public static double OVERALL_POWER = 0.5;
    public static int DISTANCE_TO_POLE = 2100;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(60, 36, Math.toRadians(180)));

        Trajectory goToPole = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(225)))
                .build();

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            /* --------------- GO TO POLE --------------- */
            drive.followTrajectory(goToPole);

            /* --------------- ALIGN TO POLE --------------- */

            /* --------------- TURN TO CONE STACK --------------- */

        }
    }
}
