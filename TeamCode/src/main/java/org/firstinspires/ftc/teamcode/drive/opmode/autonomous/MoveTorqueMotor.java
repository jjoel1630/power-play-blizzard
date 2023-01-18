package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Config
@Autonomous(name="Move TMotor")
public class MoveTorqueMotor extends LinearOpMode {
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            timer.reset();
            while(timer.seconds() <= 2.0) {
                slide.setPower(1.0);
            }

            slide.setPower(0.0);
            break;
        }
    }
}