package org.firstinspires.ftc.teamcode.drive.opmode.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.reflect.Array;
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
    boolean checkIfSame(int pos1, int pos2, int pos3, double[] f) {
        if(f[0] == pos1 && f[1] == pos2 && f[2] == pos3) return false;
        return true;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0,0, 0);

        drive.setPoseEstimate(startPose);

//        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int prevLeftPos = drive.leftFront.getCurrentPosition();
        int prevRightPos =  drive.rightFront.getCurrentPosition();
        int prevNormalPos = drive.rightRear.getCurrentPosition();

        ArrayList<double[]> a = new ArrayList<double[]>();
        double[] g = {0, 0, 0};
        a.add(g);

        int i = 0;

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x*0.3
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("left", drive.leftFront.getCurrentPosition()-prevLeftPos);
            telemetry.addData("right", drive.rightFront.getCurrentPosition()-prevRightPos);
            telemetry.addData("normal", drive.rightRear.getCurrentPosition()-prevNormalPos);

//            if(gamepad1.a && checkIfSame(drive.leftFront.getCurrentPosition(), drive.rightFront.getCurrentPosition(), drive.rightRear.getCurrentPosition(), a.get(a.size()-1))) {
////                telemetry.addData("left", (drive.leftFront.getCurrentPosition()/8192) * 2*Math.PI*1.88976);
////                telemetry.addData("right", (drive.rightFront.getCurrentPosition()/8192) * 2*Math.PI*1.88976);
////                telemetry.addData("normal", (drive.rightRear.getCurrentPosition()/8192) * 2*Math.PI*1.88976);
////                telemetry.update();
//
//                double[] d = {drive.leftFront.getCurrentPosition(), drive.rightFront.getCurrentPosition(), drive.rightRear.getCurrentPosition()};
//                a.add(d);
//            }
//
//            for(double[] f : a) {
//                telemetry.addLine("left, right, normal: " + f[0] + " " + f[1] + " " + f[2] + "\n");
//            }

            telemetry.update();
        }
    }
}
