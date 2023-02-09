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
    /* ----------------------- TIMER, ROBOT, CAMERA INIT ----------------------- */
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;
    String webcamName = "Webcam 1";
    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    public int POSITION = 1;

    /* ----------------------- POWER VALUES FOR CAMERA ----------------------- */
    public static double straightPower = 0.3;
    public static double strafePower = 0.3;
    public static double straightTime = 1.1;
    public static double strafeTime = 1.5;
    public static double pauseTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
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

        /* ----------------------- ROBO & TIMER INIT, WAIT FOR START ----------------------- */
        robot.init(this, hardwareMap, telemetry);
        robot.resetEncoder();
        robot.runWOEncoder();

        waitForStart();

        timer = new ElapsedTime();

        /* ----------------------- CONTROL LOOP ----------------------- */
        while (!isStopRequested()) {
            telemetry.addLine("Camera color: " + POSITION);
            telemetry.update();

            /* ----------------------- DRIVE TO POSITION BASED ON NUMBER (0=LEFT, 1=STRAIGHT, 2=RIGHT) ----------------------- */
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
