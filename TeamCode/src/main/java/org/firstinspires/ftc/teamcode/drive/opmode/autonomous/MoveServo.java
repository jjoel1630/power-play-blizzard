package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@Autonomous(name="ServoMove")
public class MoveServo extends LinearOpMode {
    public Servo claw;

    public static double CLAW_MIN = 0.0;
    public static double CLAW_MAX = 0.8;

    public static int numTimes = 15;

    private ElapsedTime timer;

//    public static int openClose = 0;
//    public static double clawPos = CLAW_HOME;
//    public static double clawSpeed = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.servo.get("claw");
//        claw.setDirection(Servo.Direction.REVERSE);

        double clawPos = CLAW_MAX;
        double clawSpeed = 0.1;

        waitForStart();

        while(opModeIsActive()) {
//            if(gamepad1.right_trigger == 1.0) clawPos -= clawSpeed;
//            else if(gamepad1.left_trigger == 1.0) clawPos += clawSpeed;

//            if(openClose == 0) clawPos = 0.0;
//            if(openClose == 1) clawPos = 1.0;
//
            timer = new ElapsedTime();

            int i = 0;
            while(i < numTimes && opModeIsActive()) {
                timer.reset();
                while(timer.seconds() <= 1.5) {}

//                clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
                claw.setPosition(clawPos);

                clawPos = clawPos == CLAW_MIN ? CLAW_MAX : CLAW_MIN;

                ++i;
            }
        }
    }
}
