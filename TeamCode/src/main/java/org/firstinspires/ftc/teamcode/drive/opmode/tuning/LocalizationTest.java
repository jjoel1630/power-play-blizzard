package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0,0, 0);

        drive.setPoseEstimate(startPose);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        double[][] arr = new double[50][3];
        ArrayList<double[]> a = new ArrayList<double[]>();

        int i = 0;

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("left", drive.leftFront.getCurrentPosition());
            telemetry.addData("right", drive.rightFront.getCurrentPosition());
            telemetry.addData("normal", drive.rightRear.getCurrentPosition());

            if(gamepad1.a) {
                telemetry.addData("left", (drive.leftFront.getCurrentPosition()/8192) * 2*Math.PI*1.88976);
                telemetry.addData("right", (drive.rightFront.getCurrentPosition()/8192) * 2*Math.PI*1.88976);
                telemetry.addData("normal", (drive.rightRear.getCurrentPosition()/8192) * 2*Math.PI*1.88976);
                telemetry.update();

                double[] d = {drive.leftFront.getCurrentPosition(), drive.rightFront.getCurrentPosition(), drive.rightRear.getCurrentPosition()};
                a.add(d);
            }

            for(double[] f : a) {
                telemetry.addLine("left, right, normal: " + f[0] + " " + f[1] + " " + f[2] + "\n");
            }

            telemetry.update();
        }
    }
}
