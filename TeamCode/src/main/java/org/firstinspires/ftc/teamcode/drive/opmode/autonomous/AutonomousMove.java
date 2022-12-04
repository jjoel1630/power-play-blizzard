package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Autonomous Move")
public class AutonomousMove extends LinearOpMode {
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;

    String webcamName = "Webcam 1";

    public int POSITION = 1;
    public static double TIME = 1.1;
    public static double POWER = 0.001;
    public static double DISTANCE = 10;

    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, hardwareMap, telemetry);

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
//                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });

        waitForStart();

        timer = new ElapsedTime();

        while (!isStopRequested()) {
            robot.resetEncoder();
            robot.runWOEncoder();

            double curDistance = 0;

            while (curDistance < DISTANCE && !isStopRequested()) {
                curDistance = robot.encoderTicksToInches(robot.rightFront.getCurrentPosition());

                robot.moveOnVelo(POWER);
                telemetry.addLine("in loop: " + curDistance + " inches");
                telemetry.update();
            }
            robot.quitBot();
            double time = timer.seconds();
            while(timer.seconds() - time < 1) {}

            time = timer.seconds();

            while(timer.seconds() - time < TIME) {
                robot.strafeRight(0.2);
            }
            robot.quitBot();
            break;
        }
    }
}
