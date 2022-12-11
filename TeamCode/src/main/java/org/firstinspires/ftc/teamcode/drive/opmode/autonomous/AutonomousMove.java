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

    public int POSITION = 1;
    public static double TIME = 1.1;
    public static double POWER = 0.001;
    public static double DISTANCE = 40;

    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

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

//        PIDController controller = new PIDController(1.45, 0.145, 0.0, 14.5, timer);        PIDController controller = new PIDController(1.45, 0.145, 0.0, 14.5, timer);
        PIDController controller = new PIDController(KP, KI, KD, 14.5, timer);
//        telemetry.addLine("in loop: " + curDistance + " inches");
//        telemetry.update();

        robot.resetEncoder();
        robot.runWOEncoder();

        boolean t = false;

//        double distance = 0;
        while (!isStopRequested()) {

//            if (DISTANCE <= robot.rightFront.getCurrentPosition() || t == true) {
//                robot.quitBot();
//                t = true;
//                break;
//            }

            double power = controller.update(DISTANCE, robot.rightFront.getCurrentPosition(), timer.seconds());
            timer.reset();

            telemetry.addLine("power: " + power);
            telemetry.addLine("error: " + command + " command");

            telemetry.update();

            if(!t) {
                robot.moveOnPower(power);
            }
//            break;
        }
    }
}
