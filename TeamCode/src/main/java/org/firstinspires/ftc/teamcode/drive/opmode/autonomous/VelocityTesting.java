package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Disabled
@Config
@Autonomous(group = "drive")
public class VelocityTesting extends LinearOpMode {
    public static String motorName = "lr";
    @Override
    public void runOpMode() {
        PowerPlayBot bot = new PowerPlayBot();

        bot.init(this, hardwareMap, telemetry);

        bot.resetEncoder();
        bot.runWOEncoder();

        waitForStart();

        double maxVel = 0;

        while(opModeIsActive()) {
            if(motorName == "lr") {
                bot.leftRear.setPower(1);
                maxVel = Math.max(maxVel, bot.leftRear.getVelocity());
                telemetry.addLine("velocity: " + bot.leftRear.getVelocity());
                telemetry.update();
            } else if(motorName == "lf") {
                bot.leftFront.setPower(1);
                maxVel = Math.max(maxVel, bot.leftFront.getVelocity());
                telemetry.addLine("velocity: " + bot.leftFront.getVelocity());
                telemetry.update();
            } else if(motorName == "rr") {
                bot.rightRear.setPower(1);
                maxVel = Math.max(maxVel, bot.rightRear.getVelocity());
                telemetry.addLine("velocity: " + bot.rightRear.getVelocity());
                telemetry.update();
            } else if(motorName == "rf") {
                bot.rightFront.setPower(1);
                maxVel = Math.max(maxVel, bot.rightFront.getVelocity());
                telemetry.addLine("velocity: " + bot.rightFront.getVelocity());
                telemetry.update();
            }
        }

        telemetry.addLine("max velocity: " + maxVel);
        telemetry.update();
    }
}
