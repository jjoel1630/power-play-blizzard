package org.firstinspires.ftc.teamcode.drive.opmode.autonm4.updatedcomp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="RightSideStraight")
public class RightSideStraight extends LinearOpMode {
    /* Motor & Servo Initialization */
    DcMotorEx linearSlide;
    Servo claw;

    /* Any non static variables that will not change */
    Pose2d startPosition = new Pose2d(31.5, -64.75, Math.toRadians(90.00));
    ElapsedTime timer;

    /* Any Static variables to be customized */
    public static double CLAW_MIN = 0.5;
    public static double CLAW_MAX = 1.0;

    public static int LINEAR_SLIDE_MAX_POS = 4000;
    public static int LINEAR_SLIDE_MIN_POS = 10;
    public static double LINEAR_SLIDE_POWER = 0.7;

    public static double velConst = 15;

    public static double randomX = 25.5;
    public static double randomY = -11.5;

    public static int CAMERA_POSITION = 0;

    /* Camera Variables */
    String webcamName = "Webcam 1";
    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize Drive + Slide + Claw */
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.servo.get("claw");
        double clawPos = CLAW_MIN;

        drive.setPoseEstimate(startPosition);

        /* Trajectories for each path */
        TrajectorySequence scorePreloaded1 = drive.trajectorySequenceBuilder(startPosition)
                .lineToConstantHeading(new Vector2d(31.50, -60.00),
                        SampleMecanumDrive.getVelocityConstraint(velConst, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence scorePreloaded2 = drive.trajectorySequenceBuilder(scorePreloaded1.end())
                .lineToConstantHeading(new Vector2d(12.00, -60.00),
                        SampleMecanumDrive.getVelocityConstraint(velConst, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence scorePreloaded3 = drive.trajectorySequenceBuilder(scorePreloaded2.end())
                .lineToConstantHeading(new Vector2d(12.00, randomY),
                        SampleMecanumDrive.getVelocityConstraint(velConst, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence scorePreloaded4 = drive.trajectorySequenceBuilder(scorePreloaded3.end())
                .lineToConstantHeading(new Vector2d(randomX, randomY),
                        SampleMecanumDrive.getVelocityConstraint(velConst, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence scorePreloaded5 = drive.trajectorySequenceBuilder(scorePreloaded4.end())
                .lineToConstantHeading(new Vector2d(randomX, -5.5),
                        SampleMecanumDrive.getVelocityConstraint(velConst, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence moveBackFromPole1 = drive.trajectorySequenceBuilder(scorePreloaded4.end())
                .lineToConstantHeading(new Vector2d(randomX, randomY))
                .build();
//
        TrajectorySequence moveBackFromPole2 = drive.trajectorySequenceBuilder(moveBackFromPole1.end())
                .lineToConstantHeading(new Vector2d(12.00, randomY))
                .build();

        TrajectorySequence moveBackFromPole3 = drive.trajectorySequenceBuilder(moveBackFromPole2.end())
                .lineToConstantHeading(new Vector2d(12.00, -36.00))
                .build();

//
        TrajectorySequence parkZone0 = drive.trajectorySequenceBuilder(moveBackFromPole3.end())
                .turn(Math.toRadians(180))
                .build();
//
        TrajectorySequence parkZone1 = drive.trajectorySequenceBuilder(moveBackFromPole3.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(36.07, -35.04))
                .build();
//
        TrajectorySequence parkZone2 = drive.trajectorySequenceBuilder(moveBackFromPole3.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(58.64, -35.04))
                .build();

        /* Initialize the camera */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveColorDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /* Record the camera position */
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkPosition());
            telemetry.update();

            CAMERA_POSITION = sleeveDetection.getParkPosition();
        }

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            /* Close Claw */
            claw.setPosition(CLAW_MAX);

            timer.reset();
            while(timer.seconds() <= 1.0) {}

            /* Start to raise linear slide */
            linearSlide.setTargetPosition(LINEAR_SLIDE_MAX_POS/4);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(LINEAR_SLIDE_POWER);

            /* Start first trajectory */
            drive.followTrajectorySequence(scorePreloaded1);

            drive.followTrajectorySequence(scorePreloaded2);

            drive.followTrajectorySequence(scorePreloaded3);

            /* Finish raising linear slide */
            linearSlide.setTargetPosition(LINEAR_SLIDE_MAX_POS);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(LINEAR_SLIDE_POWER);

            drive.followTrajectorySequence(scorePreloaded4);

            /* If linear slide is not already up, wait for it */
            while(linearSlide.isBusy()) {}

            drive.followTrajectorySequence(scorePreloaded5);

            /* Drop cone */
            claw.setPosition(CLAW_MIN);

            timer.reset();
            while(timer.seconds() <= 1.0) {}

            /* Move the robot back with the second trajectory */
            drive.followTrajectorySequence(moveBackFromPole1);
            drive.followTrajectorySequence(moveBackFromPole2);

            /* Lower the linear slide */
            linearSlide.setTargetPosition(LINEAR_SLIDE_MIN_POS);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(LINEAR_SLIDE_POWER);

            drive.followTrajectorySequence(moveBackFromPole3);

            /* Move the robot to the parking zones with the third trajectory */
//            drive.followTrajectorySequence(getIntoParkingZones);

            /* Use camera value to park in the correct place */
            if(CAMERA_POSITION == 0) {
                drive.followTrajectorySequence(parkZone0);
            } else if(CAMERA_POSITION == 1) {
                drive.followTrajectorySequence(parkZone1);
            } else if(CAMERA_POSITION == 2) {
                drive.followTrajectorySequence(parkZone2);
            }

            break;
        }
    }
}
