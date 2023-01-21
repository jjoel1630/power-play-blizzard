package org.firstinspires.ftc.teamcode.drive.opmode.autonm3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class RedRightFull extends LinearOpMode {
    private ElapsedTime timer;

    public static int POSITION = 1;

    String webcamName = "Webcam 1";
    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(60, 36, Math.toRadians(180)));

        Trajectory parkFront = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 36, Math.toRadians(90)))
                .build();

        Trajectory parkLeft = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 36, Math.toRadians(90)))
                .build();

        Trajectory parkRight = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30, 36, Math.toRadians(90)))
                .build();

        /* ----------------------- CAMERA init ----------------------- */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveColorDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
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
            if(POSITION == 0) {
                drive.followTrajectory(parkLeft);
            } else if(POSITION == 1) {
                drive.followTrajectory(parkFront);
            } else if(POSITION == 2) {
                drive.followTrajectory(parkRight);
            }
        }
    }
}