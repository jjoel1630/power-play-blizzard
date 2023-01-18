package org.firstinspires.ftc.teamcode.drive.opmode.unittests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.tuning.ManualFeedforwardTuner;

@Config
@Autonomous(name="Test Slide Component")
public class LinearSlideTest extends LinearOpMode {
    public DcMotorEx slide;
    public ElapsedTime timer;

    public static int numTimes = 15;
    public static int reverse = 1;
    public static double powerTesting = 0.0;
    public static double initialPosition = 0.0;
    public static double finalPosition = 20.0;
    public static double breakTime = 10.0;
    public static String nameOfSlide = "linearSlideMain";

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    public Mode mode;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlideMain");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(reverse == 1) slide.setDirection(DcMotorSimple.Direction.REVERSE);

        mode = Mode.DRIVER_MODE;

        waitForStart();
        timer = new ElapsedTime();

//        1 rev → 4 ⅜ in → 384.5
        while(!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    if(gamepad1.a) {
                        for (int i = 0; i <= numTimes; ++i) {
                            telemetry.addLine("THE LINEAR SLIDE IS AT ITS MINIMUM POSITION, PRESS STOP TO STOP THE OPMODE. OTHERWISE WAIT " + breakTime + " SECONDS TO MOVE TO MAXIMUM.");
                            telemetry.update();

                            timer.reset();
                            while(timer.seconds() <= breakTime) {}

                            while (slide.getCurrentPosition() <= 2260) {
                                slide.setPower(powerTesting);
                                telemetry.addLine("Linear Slide Position: " + slide.getCurrentPosition());
                                telemetry.update();
                            }

                            telemetry.addLine("THE LINEAR SLIDE IS AT ITS MAXIMUM POSITION, PRESS STOP TO STOP THE OPMODE. OTHERWISE WAIT " + breakTime + " SECONDS TO MOVE TO MAXIMUM.");
                            telemetry.update();

                            timer.reset();
                            while(timer.seconds() <= breakTime) {}

                            while (slide.getCurrentPosition() >= 0) {
                                slide.setPower(-1.0 * powerTesting);
                            }
                        }
                    }

                    break;
                case DRIVER_MODE:
                    boolean linearSlowModeOn = false;

                    if(gamepad1.left_bumper) linearSlowModeOn = true;

                    double linearAxial = gamepad1.left_stick_y;
                    linearAxial = linearAxial * 1;

                    double linearPowerModifier = linearSlowModeOn ? 0.2 : 1;
                    double linearPower = linearAxial * linearPowerModifier;

                    telemetry.addData("Linear Slide Position", slide.getCurrentPosition());
                    telemetry.update();

                    slide.setPower(linearPower);

                    break;
            }
        }
    }
}
