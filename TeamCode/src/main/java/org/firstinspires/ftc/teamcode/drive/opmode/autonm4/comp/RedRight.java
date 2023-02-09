package org.firstinspires.ftc.teamcode.drive.opmode.autonm4.comp;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Red Right")
public class RedRight extends LinearOpMode {
    private ElapsedTime timer;

    public static int POSITION = 1;

    public static int strafeInches = 40;
    public static int forwardInches = 27;

    Vector2d storage = new Vector2d(63.82, -12.2);
    Pose2d storagep = new Pose2d(63.82, -12.2, 0);

//    String webcamName = "Webcam 1";
//    SleeveColorDetection sleeveDetection;
//    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(32.81, -64.87, Math.toRadians(90.00)));

        TrajectorySequence scorePreloaded = drive.trajectorySequenceBuilder(new Pose2d(32.81, -64.87, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(12.62, -53.59), Math.toRadians(90.00))
                .splineTo(new Vector2d(12.62, -17.52), Math.toRadians(90.00))
                .splineTo(new Vector2d(19.00, -4.01), Math.toRadians(45.00))
                .build();

        TrajectorySequence goToStorage1 = drive.trajectorySequenceBuilder(new Pose2d(19.00, -4.01, Math.toRadians(45.00)))
                .lineToConstantHeading(new Vector2d(12.17, -20.49))
                .build();

        TrajectorySequence goToStorage2 = drive.trajectorySequenceBuilder(new Pose2d(12.17, -20.49, Math.toRadians(45)))
                .splineTo(storage, Math.toRadians(0))
                .build();

        TrajectorySequence scoreStorage1 = drive.trajectorySequenceBuilder(new Pose2d(63.84, -12.62, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(8.31, -11.88, Math.toRadians(0)))
                .build();

        TrajectorySequence scoreStorage2 = drive.trajectorySequenceBuilder(new Pose2d(8.31, -11.88, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(19.60, -4.60, Math.toRadians(45)))
                .build();

        TrajectorySequence positionPark1 = drive.trajectorySequenceBuilder(new Pose2d(19.60, -4.60, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(11.58, -15.44, Math.toRadians(90)))
                .build();

        TrajectorySequence positionPark2 = drive.trajectorySequenceBuilder(new Pose2d(11.58, -15.44, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(12.62, -35.93, Math.toRadians(180)))
                .build();

        TrajectorySequence park0 = drive.trajectorySequenceBuilder(new Pose2d(12.62, -35.93, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(new Pose2d(12.62, -35.93, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(36.22, -35.63))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(new Pose2d(12.62, -35.93, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(59.68, -35.33))
                .build();

        TrajectorySequence onePathStorageScorev1 = drive.trajectorySequenceBuilder(storagep)
                .lineToLinearHeading(new Pose2d(43.36, -13.69, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(28.56, -5.21, Math.toRadians(135.00)))
                .build();
        TrajectorySequence onePathStorageGo1 = drive.trajectorySequenceBuilder(new Pose2d(28.56, -5.21, Math.toRadians(135.00)))
                .lineToLinearHeading(new Pose2d(43.36, -13.69, Math.toRadians(0.00)))
                .lineToLinearHeading(storagep)
                .build();

        /* ----------------------- CAMERA init ----------------------- */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
//        sleeveDetection = new SleeveColorDetection();
//        camera.setPipeline(sleeveDetection);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });

        /* ----------------------- GET POSITION ----------------------- */
//        while (!isStarted()) {
//            telemetry.addData("ROTATION: ", sleeveDetection.getParkPosition());
//            telemetry.update();
//
//            POSITION = sleeveDetection.getParkPosition();
//        }

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            drive.followTrajectorySequence(scorePreloaded);
            drive.followTrajectorySequence(goToStorage1);
            drive.followTrajectorySequence(goToStorage2);
//            drive.followTrajectorySequence(scoreStorage1);
//            drive.followTrajectorySequence(scoreStorage2);
//            drive.followTrajectorySequence(positionPark1);
//            drive.followTrajectorySequence(positionPark2);
            drive.followTrajectorySequence(onePathStorageScorev1);
            drive.followTrajectorySequence(onePathStorageGo1);

//            if(POSITION == 0) {
//                drive.followTrajectorySequence(park0);
//            } else if(POSITION == 1) {
//                drive.followTrajectorySequence(park1);
//            } else if(POSITION == 2) {
//                drive.followTrajectorySequence(park2);
//            }

            break;
        }
    }
}
