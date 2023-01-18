package org.firstinspires.ftc.teamcode.drive.opmode.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PIDController;

@Autonomous(name="Test Slide Component")
public class LinearSlideTest extends LinearOpMode {
    DcMotorEx slide;
    ElapsedTime timer;

    public static int numTimes = 15;
    public static int reverse = 1;
    public static String nameOfSlide = "linearSlideMain"

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlideMain");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(reverse == 1) slide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        timer = new ElapsedTime();

//        1 rev → 4 ⅜ in → 384.5
        while(!isStopRequested()) {
            for (int i = 0; i <= numTimes; ++i) {
                while(slide.getCurrentPosition() <= 2260) {
                    slide.setPower(1.0);
                    telemetry.addLine("Linear Slide Position: " + slide.getCurrentPosition());
                    telemetry.update();
                }

                timer.reset();
                while(timer.seconds() <= 1.5) {}

                while(slide.getCurrentPosition() >= 0) {
                    slide.setPower(-1.0);
                }
            }

            break;
        }
    }
}
