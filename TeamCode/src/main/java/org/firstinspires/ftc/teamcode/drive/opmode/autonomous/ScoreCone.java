package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Config
@Autonomous(name="Score Cone", group="scorecone")
public class ScoreCone extends LinearOpMode {
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;
    DcMotorEx slide;

    public static int DISTANCE_TO_POLE = 2265;
    public static double POWER_TO_POLE = 0.5;
    public static double STRAFE_TIME = 0.5;
    public static double STRAFE_POWER_TO_POLE = 0.5;
    public static int STRAFE_POS = 530;
    public static int SLIDE_POS = 4000;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        timer = new ElapsedTime();

        robot.init(this, hardwareMap, telemetry);

        robot.resetEncoder();

        while (!isStopRequested()) {
            robot.setMotorTargetPos(DISTANCE_TO_POLE);
            robot.moveOnPower(POWER_TO_POLE);
            robot.runToPositionMode();

            while(robot.isBusy()) {}
            robot.quitBot();

            timer.reset();
            while(timer.seconds() <= 1.0) {}

            robot.resetEncoder();
//            robot.runWOEncoder();
            timer.reset();


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

            break;
        }
    }
}
