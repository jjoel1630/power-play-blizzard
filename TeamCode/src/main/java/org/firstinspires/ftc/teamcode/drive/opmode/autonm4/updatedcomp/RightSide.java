package org.firstinspires.ftc.teamcode.drive.opmode.autonm4.updatedcomp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="RightSide")
public class RightSide extends LinearOpMode {
    /* Motor & Servo Initialization */
    DcMotorEx linearSlide;
    Servo claw;

    /* Any non static variables that will not change */
    Pose2d startPosition = new Pose2d(31.18, -64.73, Math.toRadians(90.00));

    /* Any Static variables to be customized */
    public static double CLAW_MIN = 0.5;
    public static double CLAW_MAX = 1.0;

    public static int LINEAR_SLIDE_MAX_POS = 3600;
    public static int LINEAR_SLIDE_MIN_POS = 10;
    public static double LINEAR_SLIDE_POWER = 0.4;

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

        /* Trajectories for each path */
        TrajectorySequence scorePreloaded = drive.trajectorySequenceBuilder(startPosition)
                .splineToConstantHeading(new Vector2d(12.47, -56.71), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.47, -24.49), Math.toRadians(90.00))
                .splineTo(new Vector2d(20.93, -3.56), Math.toRadians(45.00))
                .build();

        TrajectorySequence moveBackFromPole = drive.trajectorySequenceBuilder(scorePreloaded.end())
                .lineToConstantHeading(new Vector2d(12.47, -14.25))
                .build();

        TrajectorySequence getIntoParkingZones = drive.trajectorySequenceBuilder(moveBackFromPole.end())
                .lineToLinearHeading(new Pose2d(12.47, -35.04, Math.toRadians(90.00)))
                .build();

        TrajectorySequence parkZone0 = drive.trajectorySequenceBuilder(getIntoParkingZones.end())
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence parkZone1 = drive.trajectorySequenceBuilder(getIntoParkingZones.end())
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(36.07, -35.04))
                .build();

        TrajectorySequence parkZone2 = drive.trajectorySequenceBuilder(getIntoParkingZones.end())
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

        while(opModeIsActive()) {
            /* Start to raise linear slide */
            linearSlide.setTargetPosition(LINEAR_SLIDE_MAX_POS);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(LINEAR_SLIDE_POWER);

            /* Close Claw */
            claw.setPosition(CLAW_MAX);

            /* Start first trajectory */
            drive.followTrajectorySequence(scorePreloaded);

            /* If linear slide is not already up, wait for it */
            while(linearSlide.isBusy()) {}

            /* Drop cone */
            claw.setPosition(CLAW_MIN);

            /* Move the robot back with the second trajectory */
            drive.followTrajectorySequence(moveBackFromPole);

            /* Lower the linear slide */
            linearSlide.setTargetPosition(LINEAR_SLIDE_MIN_POS);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(LINEAR_SLIDE_POWER);

            /* Move the robot to the parking zones with the third trajectory */
            drive.followTrajectorySequence(getIntoParkingZones);

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
