package org.firstinspires.ftc.teamcode.drive.opmode.unittests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ClawLinearAuton extends LinearOpMode {
    DcMotorEx linearSlide;
    Servo claw;

    public ElapsedTime timer;

    public static String clawName = "claw";
    public static String slideName = "linearSlide";

    public static int reverse = 1; // 1 = motor reversed, 0 = normal
    public static double slidePower = 0.0; // power for motor between -1.0 to 1.0
    public static double initialPosition = 0.0; // initial position of the linear slide (in encoder ticks)
    public static double highestPosition = 20.0; // final position of the linear slide (in encoder ticks) max: 2260;
    public static double slideTicksOneCone = 10.0; // how many ticks the linear slide must be raised up to pick up one cone;
    public static double breakTime = 10.0; // break time between up and down (in seconds)

    @Override
    public void runOpMode() throws InterruptedException {
        linearSlide = hardwareMap.get(DcMotorEx.class, slideName);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(reverse == 1) linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        timer = new ElapsedTime();

        while(opModeIsActive()) {
            timer.reset();

            // close claw

            for(int i = 5; i >= 1; --i) {
                raiseToPos(highestPosition);

                // openClaw

                lowerToPos(slideTicksOneCone*i);

                // pause + align dt

                // close claw
            }
        }
    }

    public void raiseToPos(double ticks) {
        while(linearSlide.getCurrentPosition() <= ticks) {
            linearSlide.setPower(slidePower);
            telemetry.addLine("Position of " + slideName + ": " + linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0.0);
    }

    public void lowerToPos(double ticks) {
        while(linearSlide.getCurrentPosition() >= ticks) {
            linearSlide.setPower(-1*slidePower);
            telemetry.addLine("Position of " + slideName + ": " + linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0.0);
    }
}
