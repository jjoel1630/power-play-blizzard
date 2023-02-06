package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@TeleOp(name="Servo", group="Linear OpMode")
public class ServoTesting extends LinearOpMode {
    private Servo claw;

    private double CLAW_HOME = 0.0;
    private double CLAW_MIM = 0.0;
    private double CLAW_MAX = 1.0;

    @Override
    public void runOpMode() {
        double left, right;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        claw = hardwareMap.servo.get("claw");

        claw.setPosition(CLAW_HOME);

        double clawPos = CLAW_HOME;
        double clawSpeed = 0.03;

        waitForStart();

        while (!isStopRequested()) {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            if(gamepad1.a) clawPos += clawSpeed;
            else if(gamepad1.b) clawPos -= clawSpeed;

            clawPos = Range.clip(clawPos, CLAW_MIM, CLAW_MAX);
            claw.setPosition(clawPos);

            telemetry.addData("claw", "%.2f", clawPos);

            sleep(40);
        }
    }
}
