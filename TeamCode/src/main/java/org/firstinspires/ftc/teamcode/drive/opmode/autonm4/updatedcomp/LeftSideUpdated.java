package org.firstinspires.ftc.teamcode.drive.opmode.autonm4.updatedcomp;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveDetectionLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="LeftSideUpdated")
public class LeftSideUpdated extends LinearOpMode {
    private ElapsedTime timer;

    public static int POSITION = 1;

    public static int strafeInches = 40;
    public static int forwardInches = 27;

    String webcamName = "Webcam 1";
    SleeveDetectionLeft sleeveDetection;
    OpenCvCamera camera;

    DcMotorEx linearSlide;
    Servo claw;

    public static double CLAW_MIN = 0.80;
    public static double CLAW_MAX = 1.0;

    public static double x = -21.0; // 19.00
    public static double y = -6.0; // 4.01

    Vector2d storage = new Vector2d(-63.82, -12.2);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        claw = hardwareMap.servo.get("claw");

        double clawPos = CLAW_MAX;

        drive.setPoseEstimate(new Pose2d(-32.81, -64.87, Math.toRadians(90.00)));

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(new Pose2d(-32.81, -64.87, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-32.81, -35.87))
                .lineToConstantHeading(new Vector2d(-53.81, -35.87))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(new Pose2d(-32.81, -64.87, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-32.81, -35.87))
                .build();

        TrajectorySequence park0 = drive.trajectorySequenceBuilder(new Pose2d(-32.81, -64.87, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-32.81, -35.87))
                .lineToConstantHeading(new Vector2d(-8.00, -35.87))
                .build();


        /* ----------------------- CAMERA init ----------------------- */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetectionLeft();
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

        /* ----------------------- GET POSITION ----------------------- */
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkPosition());
            telemetry.update();

            POSITION = sleeveDetection.getParkPosition();
        }

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
//            linearSlide.setTargetPosition(500);
//            linearSlide.setPower(0.4);
//            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
//            claw.setPosition(clawPos);

            if(POSITION == 0) {
                drive.followTrajectorySequence(park2);
            } else if(POSITION == 1) {
                drive.followTrajectorySequence(park1);
            } else if(POSITION == 2) {
                drive.followTrajectorySequence(park0);
            }

            break;
        }
    }
}
