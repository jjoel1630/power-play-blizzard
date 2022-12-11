package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name="Move Linear Slide")
public class MoveLinearSlide extends LinearOpMode {
    DcMotorEx slide;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        1 rev → 4 ⅜ in → 384.5
        while(!isStopRequested()) {
            slide.setTargetPosition(1318);
            slide.setPower(0.7);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            break;
        }
    }
}