package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Config
@Autonomous(name="Auton Cycle Right", group="Goated Auton")
public class AutonCycleRight extends LinearOpMode {
    /* --------------- GLOBAL VARS --------------- */
    PowerPlayBot robot = new PowerPlayBot();
    ElapsedTime timer;

    public static double OVERALL_POWER = 0.5;
    public static int DISTANCE_TO_POLE = 2100;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, hardwareMap, telemetry);

        waitForStart();

        timer = new ElapsedTime();

        while(opModeIsActive()) {
            /* --------------- GO TO POLE --------------- */
            robot.resetEncoder();
            robot.setMotorTargetPos(DISTANCE_TO_POLE);
            robot.moveOnPower(OVERALL_POWER);
            robot.runToPositionMode();

            while (robot.isBusy()) {}
            robot.quitBot();

            /* --------------- ALIGN TO POLE --------------- */

            /* --------------- TURN TO CONE STACK --------------- */

        }
    }
}
