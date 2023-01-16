package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name="two bar testing")
public class TwoBarTesting extends LinearOpMode {
    private Servo rightBar;
    private Servo leftBar;

    public static double RBAR_MAX = 1.0;
    public static double LBAR_MAX = 1.0;
    public static double RBAR_MIN = 0.0;
    public static double LBAR_MIN = 0.0;

    public double rightBarPos = RBAR_MAX;
    public double leftBarPos = LBAR_MAX;

    @Override
    public void runOpMode() throws InterruptedException {
        rightBar = hardwareMap.servo.get("rightBar");
        leftBar = hardwareMap.servo.get("leftBar");

        rightBarPos = Range.clip(rightBarPos, RBAR_MIN, RBAR_MAX);
        rightBar.setPosition(rightBarPos);

        leftBarPos = Range.clip(leftBarPos, LBAR_MIN, LBAR_MAX);
        leftBar.setPosition(leftBarPos);

        waitForStart();

        while(opModeIsActive()) {
            /* ---------------------- 2-BAR MOVEMENT ---------------------- */
//            double servoAxial = gamepad2.right_stick_y;
//            servoAxial = (servoAxial + 1.0) / 2.0;
//
//            rightBarPos = Range.clip(servoAxial, RBAR_MIN, RBAR_MAX);
//            rightBar.setPosition(rightBarPos);
//
//            leftBarPos = Range.clip(1.0-servoAxial, LBAR_MIN, LBAR_MAX);
//            leftBar.setPosition(leftBarPos);
//

            if(gamepad2.x) rightBar.setPosition(RBAR_MIN);
            if(gamepad2.y) {
                rightBarPos = RBAR_MAX;
                leftBarPos = LBAR_MIN;
            }
            else if(gamepad2.a) {
                rightBarPos = RBAR_MIN;
                leftBarPos = LBAR_MAX;
            }

            rightBarPos = Range.clip(rightBarPos, RBAR_MIN, RBAR_MAX);
            rightBar.setPosition(rightBarPos);

            leftBarPos = Range.clip(leftBarPos, LBAR_MIN, LBAR_MAX);
            leftBar.setPosition(leftBarPos);
//
//            clawBarPos = Range.clip(leftBarPos*1.6, 0.0, 1.0);
//            clawBar.setPosition(clawBarPos);

            telemetry.addLine("Right pos" + rightBarPos);
            telemetry.addLine("left pos" + leftBarPos);
            telemetry.update();

        }
    }
}
