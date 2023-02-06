package org.firstinspires.ftc.teamcode.drive.opmode.autonm4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(name="pathfinding testing")
public class PathFindingTesting extends LinearOpMode {
    public static double splineTangent = 90;
    public static Vector2d v1 = new Vector2d(-32.275, 64.25);
    public static Vector2d v2 = new Vector2d(-8.775, 64.25);
    public static Vector2d v3 = new Vector2d(-8.775, 17.25);
    public static Vector2d v4 = new Vector2d(-24, 17.25);
    public static Vector2d v5 = new Vector2d(-24, 4);

    public static Pose2d storage = new Pose2d(-40, 12, Math.toRadians(180));
    public static Vector2d storagev = new Vector2d(-40, 12);

    DcMotorEx linearSlide;

    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        Pose2d current = new Pose2d(-32.275, 66.25, Math.toRadians(270));
        drive.setPoseEstimate(current);

        // SCORE PRELOADED CONE
        Trajectory goToPolePreloaded1 = drive.trajectoryBuilder(current)
                .lineToConstantHeading(v1)
                .build();
        Trajectory goToPolePreloaded2 = drive.trajectoryBuilder(goToPolePreloaded1.end())
                .lineToConstantHeading(v2)
                .build();
        Trajectory goToPolePreloaded3 = drive.trajectoryBuilder(goToPolePreloaded2.end())
                .lineToConstantHeading(v3)
                .build();
        Trajectory goToPolePreloaded4 = drive.trajectoryBuilder(goToPolePreloaded3.end())
                .lineToConstantHeading(v4)
                .build();
        Trajectory goToPolePreloaded5 = drive.trajectoryBuilder(goToPolePreloaded4.end())
                .lineToConstantHeading(v5)
                .build();

        // RESET ORIGIN AFTER SCORE POLE SINCE ROBOT MAY HAVE HIT POLE AND RECORDED DIFFERENT DISTANCE FROM ROADRUNNER
        current = goToPolePreloaded5.end();

        // GO TO STORAGE AREA
        Trajectory goToStorage1 = drive.trajectoryBuilder(current)
                .lineToConstantHeading(new Vector2d(-24, 20))
                .build();
        Trajectory goToStorage2 = drive.trajectoryBuilder(goToStorage1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
//                .splineToSplineHeading(storage, Math.toRadians(splineTangent))
                .lineToConstantHeading(storagev)
                .build();


        waitForStart();

        while(opModeIsActive()) {
//            linearSlide.setTargetPosition(1000);
//            linearSlide.setPower(0.6);
//            linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//            telemetry.addData("left", drive.leftFront.getCurrentPosition());
//            telemetry.addData("right", drive.leftRear.getCurrentPosition());
//            telemetry.addData("normal", drive.rightRear.getCurrentPosition());
//            telemetry.addData("range", String.format("%.01f inch", distanceSensor.getDistance(DistanceUnit.INCH)));
//            telemetry.update();

            drive.followTrajectory(goToPolePreloaded1);
            drive.followTrajectory(goToPolePreloaded2);
            drive.followTrajectory(goToPolePreloaded3);
            drive.followTrajectory(goToPolePreloaded4);
            drive.followTrajectory(goToPolePreloaded5);

            drive.followTrajectory(goToStorage1);
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(goToStorage2);

            telemetry.addData("range", String.format("%.01f inch", distanceSensor.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
