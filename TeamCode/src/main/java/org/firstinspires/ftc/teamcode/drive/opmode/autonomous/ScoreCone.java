package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Config
@Autonomous(name="Score Cone", group="scorecone")
public class ScoreCone extends LinearOpMode {
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;

    public static int DISTANCE_TO_POLE = 40;
    public static double POWER_TO_POLE = 0.5;
    public static double STRAFE_TIME = 0.5;
    public static double STRAFE_POWER_TO_POLE = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        timer = new ElapsedTime();

        robot.resetEncoder();

        while (!isStopRequested()) {
            robot.setMotorTargetPos(DISTANCE_TO_POLE);
            robot.moveOnPower(POWER_TO_POLE);
            robot.runToPositionMode();

            timer.reset();
            while(timer.seconds() <= 1.0) {}

            robot.resetEncoder();
            robot.runWOEncoder();
            timer.reset();
            while(timer.seconds() <= STRAFE_TIME) {
                robot.strafeLeft(STRAFE_POWER_TO_POLE);
            }

            break;
        }
    }
}
