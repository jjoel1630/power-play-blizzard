package org.firstinspires.ftc.teamcode.drive.opmode.autonm4.updatedcomp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="RightSideTest")
public class RightSideTest extends LinearOpMode {
    /* Motor & Servo Initialization */
    DcMotorEx linearSlide;
    Servo claw;

    ElapsedTime timer;

    /* Any non static variables that will not change */
    Pose2d startPosition = new Pose2d(31.5, -66.00, Math.toRadians(90.00));

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

        drive.setPoseEstimate(startPosition);

        /* Trajectories for each path */
        TrajectorySequence scorePreloaded1 = drive.trajectorySequenceBuilder(startPosition)
                .splineToConstantHeading(new Vector2d(12.47, -56.71), Math.toRadians(90.00))
                .build();

        TrajectorySequence scorePreloaded2 = drive.trajectorySequenceBuilder(scorePreloaded1.end())
                .splineTo(new Vector2d(12.47, -24.49), Math.toRadians(90.00))
                .build();

        TrajectorySequence scorePreloaded3 = drive.trajectorySequenceBuilder(scorePreloaded2.end())
                .splineTo(new Vector2d(20.5, -6.05), Math.toRadians(45.00))
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
            /* Start first trajectory */
            drive.followTrajectorySequence(scorePreloaded1);

            timer.reset();
            while(timer.seconds() <= 5.0) {}

            drive.followTrajectorySequence(scorePreloaded2);

            timer.reset();
            while(timer.seconds() <= 5.0) {}

            drive.followTrajectorySequence(scorePreloaded3);

            break;
        }
    }
}
