package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Config
@Autonomous(name="ScoreConePID", group="scorecone")
public class ScoreConePID extends LinearOpMode {
    /* ----------------------- TIMER, ROBO INIT ----------------------- */
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;

    /* ----------------------- PID TUNING INIT ----------------------- */
    public static int targetPosition = 5000;

    public static double[] KrightFront = {1.0, 0.0, 0.0};
    public static double[] KrightRear = {1.0, 0.0, 0.0};
    public static double[] KleftFront = {1.0, 0.0, 0.0};
    public static double[] KleftRear = {1.0, 0.0, 0.0};

    public double lastError = 0.0;
    public double integralSum = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        /* ----------------------- ROBOT INIT  ----------------------- */
        robot.init(this, hardwareMap, telemetry);

        robot.resetEncoder();
        robot.runWOEncoder();

        waitForStart();

        /* ----------------------- CONTROL LOOP ----------------------- */
        while(!isStopRequested()) {
            /* ----------------------- PID FUNCTIONS TO POWER ----------------------- */
            double rightFrontPower = returnPower(targetPosition, robot.rightFront.getCurrentPosition(), KrightFront);
            double rightRearPower = returnPower(targetPosition, robot.rightRear.getCurrentPosition(), KrightRear);
            double leftFrontPower = returnPower(targetPosition, robot.leftFront.getCurrentPosition(), KleftFront);
            double leftRearPower = returnPower(targetPosition, robot.leftRear.getCurrentPosition(), KleftRear);

            telemetry.addLine("right front power: " + rightFrontPower);
            telemetry.addLine("left front power: " + leftFrontPower);
            telemetry.addLine("right rear power: " + rightRearPower);
            telemetry.addLine("left rear power: " + leftRearPower);
            telemetry.update();

            robot.rightFront.setPower(rightFrontPower);
            robot.rightRear.setPower(rightRearPower);
            robot.leftFront.setPower(leftFrontPower);
            robot.leftRear.setPower(leftRearPower);
        }
    }

    /* ----------------------- PID FUNCTION ----------------------- */
    public double returnPower(int target, int current, double[] kvals) {
        double err = target - current;
        integralSum += err*timer.seconds();
        double derivative = (err-lastError) / timer.seconds();
        lastError = err;

        timer.reset();

        return ((err*kvals[0]) + (derivative * kvals[2]) + (integralSum * kvals[1]));
    }
}
