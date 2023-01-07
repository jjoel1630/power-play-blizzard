package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="Move Linear Slide")
public class MoveLinearSlide extends LinearOpMode {
    DcMotorEx slide;

    public static int upOrDown = 1;
    public static int UPDISTANCE = 1;
    public static int DOWNDISTANCE = 1;
    public static int DISTANCE = 0;
    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static int numTimes = 15;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDController controller = new PIDController(KP, KI, KD, 14.5, timer);

        waitForStart();
        timer = new ElapsedTime();

//        1 rev → 4 ⅜ in → 384.5
        while(!isStopRequested()) {
//            if(upOrDown == 1) {
//                slide.setTargetPosition(UPDISTANCE);
//                slide.setPower(0.7);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                telemetry.addLine("position: " + slide.getCurrentPosition());
//                telemetry.update();
//            } else {
//                slide.setTargetPosition(DOWNDISTANCE);
//                slide.setPower(-0.5);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                telemetry.addLine("position: " + slide.getCurrentPosition());
//                telemetry.update();
//            }
//            telemetry.addLine("position: " + slide.getCurrentPosition());
//            telemetry.update();
//            double[] pwr = controller.update(DISTANCE, slide.getCurrentPosition());
//
//            slide.setPower(pwr[0]);
//
//            telemetry.addLine("power: " + pwr[0]);
//            telemetry.addLine("error: " + pwr[1]);
//
//            telemetry.update();
            for (int i = 0; i <= numTimes; ++i) {
                while(slide.getCurrentPosition() <= 2260) {
                    slide.setPower(1.0);
                    telemetry.addLine("position: " + slide.getCurrentPosition());
                    telemetry.update();
                }

                timer.reset();
                while(timer.seconds() <= 1.5) {}

                while(slide.getCurrentPosition() >= 0) {
                    slide.setPower(-1.0);
                }
            }
        }
    }
}