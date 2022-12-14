package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robocol.Command;
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

    public static int POSITION = 1;
    public double TIME = 1.1;
    public double POWER = 0.001;
    public double DISTANCE = 40;

    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    PIDController controller;
    public double KP = 0.0;
    public double KI = 0.0;
    public double KD = 0.0;

    public static double straightPower = 0.3;
    public static double strafePower = 0.3;
    public static double straightTime = 1.1;
    public static double strafeTime = 1.5;
    public static double pauseTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
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

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkPosition());
            telemetry.update();

            POSITION = sleeveDetection.getParkPosition();
        }

        robot.init(this, hardwareMap, telemetry);

        waitForStart();

        timer = new ElapsedTime();
//        controller = new PIDController(KP, KI, KD, 14.5, timer);

        robot.resetEncoder();
        robot.runWOEncoder();

        while (!isStopRequested()) {
            telemetry.addLine("Camera color: " + POSITION);
            telemetry.update();

//            straightPower = 0.3;
//            double strafePower = 0.3;
//            double straightTime = 1.0;
//            double strafeTime = 1.8;
//            double pauseTime = 1.0;

            if(POSITION == 1) {
                timer.reset();
                while(timer.seconds() <= straightTime)
                    robot.moveOnPower(straightPower);
            } else if(POSITION == 2) {
                timer.reset();
                while(timer.seconds() <= straightTime)
                    robot.moveOnPower(straightPower);

                timer.reset();
                while(timer.seconds() <= pauseTime) {}

                timer.reset();
                while(timer.seconds() <= strafeTime)
                    robot.strafeRight(strafePower);
            } else {
                timer.reset();
                while(timer.seconds() <= straightTime)
                    robot.moveOnPower(straightPower);

                timer.reset();
                while(timer.seconds() <= pauseTime) {}

                timer.reset();
                while(timer.seconds() <= strafeTime)
                    robot.strafeLeft(strafePower);
            }

            break;
        }
    }
}
