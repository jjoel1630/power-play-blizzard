package org.firstinspires.ftc.teamcode.drive.opmode.autonm3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name="Red Right Test Two", group="Neat Unit Tests")
public class RedRightTestTwo extends LinearOpMode {
    ElapsedTime timer;

    private DcMotorEx linearSlide = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(60, 36, Math.toRadians(180));
        Pose2d storageArea = new Pose2d(12, 36, Math.toRadians(90));
        Pose2d scoringPole = new Pose2d(0, 24, Math.toRadians(225));

        drive.setPoseEstimate(startPose);

        Trajectory goToStorage = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(storageArea)
                .build();

        Trajectory goToPolePreloaded = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 24, Math.toRadians(225)))
                .build();

        Trajectory goToPoleStorage = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 24, Math.toRadians(225)))
                .build();

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            /* --------------- GO TO POLE --------------- */
            drive.followTrajectory(goToPolePreloaded);
            drive.followTrajectory(goToStorage);
            drive.followTrajectory(goToPoleStorage);

//            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            linearSlide.setPower(0.1);
//            linearSlide.setTargetPosition(1000);
        }
    }
}
