package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="double linear slide testing")
public class DoubleLinearSlideTest extends LinearOpMode {
    DcMotorEx slideRight;
    DcMotorEx slideLeft;

    public static int numTimes = 15;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        slideRight = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        slideLeft = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        timer = new ElapsedTime();

//        1 rev → 4 ⅜ in → 384.5
        while(!isStopRequested()) {
            for (int i = 0; i <= numTimes; ++i) {
                while(slideRight.getCurrentPosition() <= 2260 && slideLeft.getCurrentPosition() <= 2260) {
                    slideRight.setPower(0.7);
                    slideLeft.setPower(0.7);

                    telemetry.addLine("right pos: " + slideRight.getCurrentPosition());
                    telemetry.addLine("left pos: " + slideLeft.getCurrentPosition());
                    telemetry.update();
                }

                timer.reset();
                while(timer.seconds() <= 1.5) {}

                while(slideRight.getCurrentPosition() >= 0 && slideLeft.getCurrentPosition() >= 0) {
                    slideRight.setPower(-0.7);
                    slideLeft.setPower(-0.7);
                }
            }
        }
    }
}
