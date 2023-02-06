package org.firstinspires.ftc.teamcode.drive.opmode.autonm3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name="Red Right New Path", group="Neat Unit Tests")
public class RedRightNewPath extends LinearOpMode {
    ElapsedTime timer;

    public static double astartX = 36;
    public static double astartY = -60;
    public static double ainitialAngle = 90;

    public static double aStrafeDist = 52;
    public static double aForwardDist = 64;
    public static double bStrafeDist = 42;
    public static double bForwardDist = 10;
    public static double linSlide = -400;
    public static double parkingStrafe1 = 50;
    public static double parkingStrafe2 = 30;
    public static double parkingStrafe3 = 60;
    public static double cBackwardDist = 12;
    public static double forwardSlightD = 7;

    public static double POSITION = 0;
//    public static double cForwardDist = 10;
//    public static double storageAreaDist = 40;
    public static double turnAngle1 = 90;
//    public static double turnAngle2 = -90;
//    public static int coneStackEnc = -30;

//
//    public static double bstrafeX = 12;
//    public static double bstrafeY = -60;
//    public static double bstrafeAngle = 90;
//
//    public static double cpole1X = 12;
//    public static double cpole1Y = -12;
//    public static double cpole1Angle = 90;
//
//    public static double dpole2X = 20;
//    public static double dpole2Y = -12;
//    public static double dpole2Angle = 45;
//
//    public static double estorageX = 60;
//    public static double estorageY = 12;
//    public static double estorageAngle = 0;

    private DcMotorEx linearSlide = null;

    private Servo claw;

    public static double CLAW_MIN = 0.75;
    public static double CLAW_MAX = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw = hardwareMap.servo.get("claw");
        claw.setPosition(CLAW_MIN);

        Pose2d startPose = new Pose2d(astartX, astartY, Math.toRadians(ainitialAngle));
//        Vector2d moveLeftTilePose = new Vector2d(bstrafeX, bstrafeY);
//        Pose2d goToPolePose = new Pose2d(cpole1X, cpole1Y, Math.toRadians(cpole1Angle));
//        Pose2d scoreConePolePose = new Pose2d(dpole2X, dpole2Y, Math.toRadians(dpole2Angle));
//        Pose2d storageAreaPose = new Pose2d(estorageX, estorageY, Math.toRadians(estorageAngle));

        drive.setPoseEstimate(startPose);

        Trajectory forwardSlight = drive.trajectoryBuilder(startPose)
                .forward(forwardSlightD)
                .build();

        Trajectory moveLeft = drive.trajectoryBuilder(forwardSlight.end())
                .strafeLeft(aStrafeDist)
                .build();

        Trajectory goToPoleLeft = drive.trajectoryBuilder(moveLeft.end())
                .forward(aForwardDist)
                .build();

        Trajectory alignPole = drive.trajectoryBuilder(moveLeft.end())
                .strafeRight(bStrafeDist)
                .build();

        Trajectory alignPole2 = drive.trajectoryBuilder(moveLeft.end())
                .forward(bForwardDist)
                .build();

        Trajectory leavePole = drive.trajectoryBuilder(alignPole2.end())
                .back(cBackwardDist)
                .build();

//        Trajectory storageArea = drive.trajectoryBuilder(leavePole.end())
//                .forward(storageAreaDist)
//                .build();
//
//        Trajectory backToPole = drive.trajectoryBuilder(storageArea.end())
//                .back(storageAreaDist)
//                .build();

//        Trajectory alignPole3 = drive.trajectoryBuilder(backToPole.end())
//                .strafeLeft(parkingStrafe1)
//                .build();

        Trajectory alignParking1 = drive.trajectoryBuilder(leavePole.end())
                .strafeRight(parkingStrafe2)
                .build();

        Trajectory alignParking2 = drive.trajectoryBuilder(leavePole.end())
                .strafeRight(parkingStrafe3)
                .build();

        Trajectory alignParking3 = drive.trajectoryBuilder(leavePole.end())
                .forward(bForwardDist)
                .build();

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            claw.setPosition(CLAW_MAX);

            /* --------------- GO TO POLE --------------- */
            drive.followTrajectory(forwardSlight);
            drive.followTrajectory(moveLeft);
            drive.followTrajectory(goToPoleLeft);

            linearSlide.setTargetPosition(-400);
            linearSlide.setPower(0.1);
            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            drive.followTrajectory(alignPole);
//            drive.followTrajectory(goToPolePreloaded);
//            drive.followTrajectory(scorePolePreloaded);
//            drive.followTrajectory(goToStorage);
//
//            linearSlide.setTargetPosition(-100);
//            linearSlide.setPower(0.1);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            drive.followTrajectory(alignPole2);

            claw.setPosition(CLAW_MIN);

            drive.followTrajectory(leavePole);
            if(POSITION == 0) {
                drive.followTrajectory(alignParking1);
            } else if(POSITION == 1) {
                drive.followTrajectory(alignParking1);
            } else if(POSITION == 2) {
                drive.followTrajectory(alignParking1);
            }
//            drive.turn(Math.toRadians(turnAngle1));
//            linearSlide.setTargetPosition(coneStackEnc);
//            linearSlide.setPower(0.1);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            drive.followTrajectory(storageArea);
//
//            claw.setPosition(CLAW_MAX);
//            linearSlide.setTargetPosition(-100);
//            linearSlide.setPower(0.1);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//            while(linearSlide.isBusy()) {}
//
//            drive.followTrajectory(backToPole);
//            drive.turn(turnAngle2);
//            linearSlide.setTargetPosition(-400);
//            linearSlide.setPower(0.1);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            drive.followTrajectory(alignPole3);
//
//            claw.setPosition(CLAW_MIN);

            break;
        }
    }
}
