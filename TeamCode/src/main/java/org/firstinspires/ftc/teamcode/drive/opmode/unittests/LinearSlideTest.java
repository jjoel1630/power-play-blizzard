package org.firstinspires.ftc.teamcode.drive.opmode.unittests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="Test Slide Component", group="Neat Unit Tests")
public class LinearSlideTest extends LinearOpMode {
    public DcMotorEx slide;
    public ElapsedTime timer;

    public static int numTimes = 15; // number of times to run opmode
    public static int reverse = 1; // 1 = motor reversed, 0 = normal
    public static double powerTesting = 0.0; // power for motor between -1.0 to 1.0
    public static double initialPosition = 0.0; // initial position of the linear slide (in encoder ticks)
    public static double finalPosition = 20.0; // final position of the linear slide (in encoder ticks) max: 2260;
    public static double breakTime = 10.0; // break time between up and down (in seconds)
    public static String nameOfSlide = "linearSlideMain"; // name of linear slide in the driver station application

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

//        1 rev → 4 ⅜ in → 384.5 encoder ticks
        while(!isStopRequested()) {
            telemetry.addData("mode", mode);

            /* ----------------------- SWITCH B/W AUTON * TELEOP ----------------------- */
            switch (mode) {
                case TUNING_MODE:
                    // Button to switch between op modes
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    if(gamepad1.a) {
                        for (int i = 0; i <= numTimes; ++i) {
                            telemetry.addLine("" + emulateClaw(true));
                            telemetry.addLine("THE LINEAR SLIDE IS AT ITS MINIMUM POSITION, PRESS STOP TO STOP THE OPMODE. OTHERWISE WAIT " + breakTime + " SECONDS TO MOVE TO MAXIMUM.");
                            telemetry.addLine("" + emulateClaw(false));
                            telemetry.update();

                            timer.reset();
                            while(timer.seconds() <= breakTime) {}

                            while (slide.getCurrentPosition() <= finalPosition) {
                                slide.setPower(powerTesting);
                                telemetry.addLine("Linear Slide Position: " + slide.getCurrentPosition());
                                telemetry.update();
                            }

                            telemetry.addLine("" + emulateClaw(true));
                            telemetry.addLine("THE LINEAR SLIDE IS AT ITS MAXIMUM POSITION, PRESS STOP TO STOP THE OPMODE. OTHERWISE WAIT " + breakTime + " SECONDS TO MOVE TO MAXIMUM.");
                            telemetry.addLine("" + emulateClaw(false));
                            telemetry.update();

                            timer.reset();
                            while(timer.seconds() <= breakTime) {}

                            while (slide.getCurrentPosition() >= initialPosition) {
                                slide.setPower(-1.0 * powerTesting);
                                telemetry.addLine("Linear Slide Position: " + slide.getCurrentPosition());
                                telemetry.update();
                            }
                        }
                    }

                    break;
                case DRIVER_MODE:
                    // Button to switch between op modes
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                    }

                    boolean linearSlowModeOn = false;

                    if(gamepad1.left_bumper) linearSlowModeOn = true;

                    double linearAxial = gamepad1.left_stick_y;
                    linearAxial = linearAxial * 1;

                    double linearPowerModifier = linearSlowModeOn ? 0.2 : 1;
                    double linearPower = linearAxial * linearPowerModifier;

                    telemetry.addData("Current Linear Slide Position", slide.getCurrentPosition());
                    telemetry.update();

                    slide.setPower(linearPower);

                    break;
            }
        }
    }

    public String emulateClaw(boolean tf) {
        return (tf ? "open claw" : "close claw");
    }
}
