package org.firstinspires.ftc.teamcode.drive.opmode.autonm4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Disabled
@Config
@Autonomous(name="IMUHeading Testing")
public class IMUHeading extends LinearOpMode {
    public static double angle = 45;
    public static int i = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(start);

        Trajectory t1 = drive.trajectoryBuilder(start, false)
                .forward(35)
                .build();

        String s = "";

        waitForStart();

        int c = 0;

        while(opModeIsActive() && c < i) {
            telemetry.addLine("c: "+c);
            telemetry.update();

            drive.turn(Math.toRadians(angle));

            while(drive.isBusy()) {}

            telemetry.addLine("IMU Data: "+drive.getIMUHeadingCustom());
            telemetry.update();

            drive.turn(Math.toRadians(-angle));

            while(drive.isBusy()) {}

            telemetry.addLine("IMU Data: "+drive.getIMUHeadingCustom());
            telemetry.update();

//            break;
            c++;
        }
    }
}
