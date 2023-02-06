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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name="Red Right Test One", group="Neat Unit Tests")
public class RedRightTestOne extends LinearOpMode {
    ElapsedTime timer;
    
    public static double startX = 36;
    public static double startY = -63.5;
    public static double initialAngle = 90;

    public static double storageX = 36;
    public static double storageY = -12;
    public static double storageAngle = 90;

    public static double scoringX = 24;
    public static double scoringY = -6;
    public static double scoringAngle = 68;

    public static double forwardDist = 55;
    public static double forwardDist2 = 15;

    private DcMotorEx linearSlide = null;

    private Servo claw;

    public static double CLAW_MIN = 0.75;
    public static double CLAW_MAX = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw = hardwareMap.servo.get("claw");
        claw.setPosition(CLAW_MIN);

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(initialAngle));
        Pose2d scoringPole = new Pose2d(scoringX, scoringY, Math.toRadians(scoringAngle));
        Pose2d storageArea = new Pose2d(storageX, storageY, Math.toRadians(storageAngle));

        drive.setPoseEstimate(startPose);

        Trajectory goToPolePreloaded = drive.trajectoryBuilder(startPose)
                .forward(forwardDist)
                .build();

        Trajectory alignToPole = drive.trajectoryBuilder(goToPolePreloaded.end())
                .lineToLinearHeading(scoringPole)
                .build();

        Trajectory alignToPolePartTwo = drive.trajectoryBuilder(alignToPole.end())
                .forward(forwardDist2)
                .build();

//        Trajectory goToPolePreloadedNew = drive.trajectoryBuilder(startPose)
//                .splineToSplineHeading(scoringPole, Math.toRadians(0))
//                .build();
//
//        Trajectory goToPoleStorage = drive.trajectoryBuilder(goToStorage.end())
//                .lineToLinearHeading(scoringPole)
//                .build();

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        claw.setPosition(CLAW_MAX);

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            /* --------------- GO TO POLE --------------- */
            drive.followTrajectory(goToPolePreloaded);

            linearSlide.setTargetPosition(-100);
            linearSlide.setPower(0.1);
            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            drive.followTrajectory(alignToPole);

            drive.followTrajectory(alignToPolePartTwo);

            claw.setPosition(CLAW_MIN);

            break;
        }
    }
}
