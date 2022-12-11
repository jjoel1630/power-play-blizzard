package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name="ServoMove")
public class MoveServo extends LinearOpMode {
    private Servo claw;

    public static double CLAW_MIN = 0.0;
    public static double CLAW_MAX = 0.7;
    public static double CLAW_HOME = CLAW_MAX;
    double clawPos = CLAW_HOME;
    public static double clawSpeed = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.servo.get("claw");

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.right_trigger == 1.0) clawPos -= clawSpeed;
            else if(gamepad1.left_trigger == 1.0) clawPos += clawSpeed;

            clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
            claw.setPosition(clawPos);
        }
    }
}
