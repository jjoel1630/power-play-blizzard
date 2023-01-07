package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveLinearAndServo extends LinearOpMode {
    public Servo claw;
    DcMotorEx slide;

    public static double CLAW_MIN = 0.0;
    public static double CLAW_MAX = 0.8;

    public static int numTimes = 30;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.servo.get("claw");
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double clawPos = CLAW_MAX;
        double clawSpeed = 0.1;

        waitForStart();
        timer = new ElapsedTime();

        claw.setPosition(clawPos);
        clawPos = CLAW_MIN;

        while(opModeIsActive()) {
            int i = 0;
            while(i < numTimes && opModeIsActive()) {
                timer.reset();
                while(timer.seconds() <= 1.5) {}

                while(slide.getCurrentPosition() <= 2260) {
                    slide.setPower(0.7);
                    telemetry.addLine("position: " + slide.getCurrentPosition());
                    telemetry.update();
                }

                timer.reset();
                while(timer.seconds() <= 1.5) {
                    slide.setPower(0.0);
                }

                while(slide.getCurrentPosition() >= 0) {
                    slide.setPower(-0.7);
                    telemetry.addLine("position: " + slide.getCurrentPosition());
                    telemetry.update();
                }

                timer.reset();
                while(timer.seconds() <= 1.5) {
                    slide.setPower(0.0);
                }

                claw.setPosition(clawPos);

                clawPos = clawPos == CLAW_MIN ? CLAW_MAX : CLAW_MIN;

                ++i;
            }
        }
    }
}
