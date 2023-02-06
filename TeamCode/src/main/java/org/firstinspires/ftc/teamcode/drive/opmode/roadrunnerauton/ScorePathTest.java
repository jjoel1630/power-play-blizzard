package org.firstinspires.ftc.teamcode.drive.opmode.roadrunnerauton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(group = "drive")
public class ScorePathTest extends LinearOpMode {
    public static double POSX = 10;
    public static double POSY = 0;
    public static double POSHEADING = 270;

    public static double DISTANCE = 45;

    ElapsedTime timer;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        Pose2d pos = new Pose2d(POSX, POSY, Math.toRadians(POSHEADING));
        Trajectory moveToPole = drive.trajectoryBuilder(pos)
                .forward(DISTANCE)
                .build();

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            drive.followTrajectory(moveToPole);

            break;
        }

        while(!isStopRequested()) {}
        telemetry.addLine(pos.toString());
        telemetry.update();
    }
}
