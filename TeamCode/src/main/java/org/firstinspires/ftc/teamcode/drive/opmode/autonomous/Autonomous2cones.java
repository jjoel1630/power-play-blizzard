package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.drive.opmode.imagedetect.SleeveColorDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Config
@Autonomous(name="Autonomous2cone")
public class Autonomous2cones extends LinearOpMode {
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;
    DcMotorEx slide;
    private Servo claw;

    public static int DISTANCE_TO_POLE = 2265;
    public static int DISTANCE_TO_PARK = 1200;
    public static double POWER_TO_POLE = 0.3;
    public static double STRAFE_TIME = 0.5;
    public static double STRAFE_POWER_TO_POLE = 0.5;
    public static int STRAFE_POS = 540;
    public static int STRAFE_PARK_POS = 1080;
    public static int SLIDE_POS = 4250;
    public static int CLAW_IN_DISTANCE = 100;
    public static int SLIDE_POS_ONE = 100;
    public static int TURN_ENCODER_DISTANCE = 1000;

    public static int POSITION = 1;

    private double CLAW_MIN= 0.0;
    private double CLAW_MAX = 0.8;
    private double CLAW_HOME = CLAW_MIN;

    String webcamName = "Webcam 1";
    SleeveColorDetection sleeveDetection;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.servo.get("claw");

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

        double clawPos = CLAW_HOME;
        double clawSpeed = 0.08;

        clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
        claw.setPosition(clawPos);

        /* ----------------------- GET POSITION ----------------------- */
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getParkPosition());
            telemetry.update();

            POSITION = sleeveDetection.getParkPosition();
        }

        /* ----------------------- ROBO & TIMER INIT, WAIT FOR START ----------------------- */
        robot.init(this, hardwareMap, telemetry);
        robot.resetEncoder();

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            claw.setPosition(CLAW_MAX);

            timer.reset();
            while(timer.seconds() <= 1.0) {}

            while(slide.getCurrentPosition() <= SLIDE_POS_ONE) {
                slide.setPower(0.7);
                telemetry.addLine("position: " + slide.getCurrentPosition());
                telemetry.update();
            }
            slide.setPower(0.0);

            robot.setMotorTargetPos(DISTANCE_TO_POLE);
            robot.moveOnPower(POWER_TO_POLE);
            robot.runToPositionMode();

            while(robot.isBusy()) {}
            robot.quitBot();

            timer.reset();
            while(timer.seconds() <= 1.0) {}
            robot.resetEncoder();

            robot.rightFront.setTargetPosition(STRAFE_POS);
            robot.rightRear.setTargetPosition(STRAFE_POS);
            robot.leftFront.setTargetPosition(-STRAFE_POS);
            robot.leftRear.setTargetPosition(-STRAFE_POS);
            robot.strafeLeft(STRAFE_POWER_TO_POLE);
            robot.runToPositionMode();

//            while(timer.seconds() <= STRAFE_TIME) {
//                robot.strafeRight(STRAFE_POWER_TO_POLE);
//                telemetry.addLine("strafe position right front: " + robot.rightFront.getCurrentPosition());
//                telemetry.addLine("strafe position right rear: " + robot.rightRear.getCurrentPosition());
//                telemetry.addLine("strafe position left front: " + robot.leftFront.getCurrentPosition());
//                telemetry.addLine("strafe position left rear: " + robot.leftRear.getCurrentPosition());
//                telemetry.update();
//            }

            while(robot.isBusy()) {}
            robot.quitBot();

            while(slide.getCurrentPosition() <= SLIDE_POS) {
                slide.setPower(0.7);
//                telemetry.addLine("position: " + slide.getCurrentPosition());
//                telemetry.update();
            }
            slide.setPower(0.0);

            timer.reset();
            while(timer.seconds() <= 1.5) {}

            robot.resetEncoder();
            robot.setMotorTargetPos(CLAW_IN_DISTANCE);
            robot.moveOnPower(POWER_TO_POLE);
            robot.runToPositionMode();

            while(robot.isBusy()) {}
            robot.quitBot();

            claw.setPosition(CLAW_MIN);

            timer.reset();
            while(timer.seconds() <= 1.5) {}

            robot.resetEncoder();
            robot.setMotorTargetPos(-CLAW_IN_DISTANCE);
            robot.moveOnPower(POWER_TO_POLE);
            robot.runToPositionMode();

            while(robot.isBusy()) {}
            robot.quitBot();

            robot.resetEncoder();

            robot.rightFront.setTargetPosition(-STRAFE_POS);
            robot.rightRear.setTargetPosition(-STRAFE_POS);
            robot.leftFront.setTargetPosition(STRAFE_POS);
            robot.leftRear.setTargetPosition(STRAFE_POS);
            robot.strafeLeft(STRAFE_POWER_TO_POLE);
            robot.runToPositionMode();

            while(robot.isBusy()) {}
            robot.quitBot();

            while(slide.getCurrentPosition() >= (SLIDE_POS - 2260)) {
                slide.setPower(-0.7);
//                telemetry.addLine("position: " + slide.getCurrentPosition());
//                telemetry.update();
            }
            slide.setPower(0.0);

            robot.resetEncoder();

            robot.rightFront.setTargetPosition(-TURN_ENCODER_DISTANCE);
            robot.rightRear.setTargetPosition(TURN_ENCODER_DISTANCE);
            robot.leftFront.setTargetPosition(TURN_ENCODER_DISTANCE);
            robot.leftRear.setTargetPosition(-TURN_ENCODER_DISTANCE);
            robot.rightFront.setPower(-0.3);
            robot.leftFront.setPower(0.3);
            robot.rightRear.setPower(0.3);
            robot.leftRear.setPower(-0.3);
            robot.runToPositionMode();

            while(robot.isBusy()) {}
            robot.quitBot();

//            robot.resetEncoder();
//            robot.setMotorTargetPos(-DISTANCE_TO_PARK);
//            robot.moveOnPower(POWER_TO_POLE);
//            robot.runToPositionMode();
//
//            while(robot.isBusy()) {}
//            robot.quitBot();
//
//            if(POSITION == 0) {
//                robot.resetEncoder();
//
//                robot.rightFront.setTargetPosition(STRAFE_PARK_POS);
//                robot.rightRear.setTargetPosition(STRAFE_PARK_POS);
//                robot.leftFront.setTargetPosition(-STRAFE_PARK_POS);
//                robot.leftRear.setTargetPosition(-STRAFE_PARK_POS);
//                robot.strafeLeft(STRAFE_POWER_TO_POLE);
//                robot.runToPositionMode();
//
//                while(robot.isBusy()) {}
//                robot.quitBot();
//            } else if(POSITION == 2) {
//                robot.resetEncoder();
//
//                robot.rightFront.setTargetPosition(-STRAFE_PARK_POS);
//                robot.rightRear.setTargetPosition(-STRAFE_PARK_POS);
//                robot.leftFront.setTargetPosition(STRAFE_PARK_POS);
//                robot.leftRear.setTargetPosition(STRAFE_PARK_POS);
//                robot.strafeLeft(STRAFE_POWER_TO_POLE);
//                robot.runToPositionMode();
//
//                while(robot.isBusy()) {}
//                robot.quitBot();
//            }
//
//            while(slide.getCurrentPosition() >= 0) {
//                slide.setPower(-0.7);
////                telemetry.addLine("position: " + slide.getCurrentPosition());
////                telemetry.update();
//            }
//            slide.setPower(0.0);
//
            break;
        }
    }
}
