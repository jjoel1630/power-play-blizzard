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
    private ElapsedTime timer = new ElapsedTime();

    /* ----------------------- PID TUNING INIT ----------------------- */
    public static int targetPosition = 5000;

    public static double PRF = 5.0;
    public static double IRF = 4.0;
    public static double DRF = 0.0;
    public static double PRR = 5.0;
    public static double IRR = 4.0;
    public static double DRR = 0.0;
    public static double PLF = 5.0;
    public static double ILF = 4.0;
    public static double DLF = 0.0;
    public static double PLR = 5.0;
    public static double ILR = 4.0;
    public static double DLR = 0.0;

    public static double[] KrightFront = {PRF, IRF, DRF};
    public static double[] KrightRear = {PRR, IRR, DRR};
    public static double[] KleftFront = {PLF, ILF, DLF};
    public static double[] KleftRear = {PLR, ILR, DLR};

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
            double[] rightFrontPower = returnPower(targetPosition, robot.rightFront.getCurrentPosition(), KrightFront);
            double[] rightRearPower = returnPower(targetPosition, robot.rightRear.getCurrentPosition(), KrightRear);
            double[] leftFrontPower = returnPower(targetPosition, robot.leftFront.getCurrentPosition(), KleftFront);
            double[] leftRearPower = returnPower(targetPosition, robot.leftRear.getCurrentPosition(), KleftRear);

            telemetry.addLine("right front pos: " + rightFrontPower[0] + " error: " + rightFrontPower[1]);
            telemetry.addLine("left front pos: " + leftFrontPower[0] + " error: " + leftFrontPower[1]);
            telemetry.addLine("right rear pos: " + rightRearPower[0] + " error: " + rightRearPower[1]);
            telemetry.addLine("left rear pos: " + leftRearPower[0] + " error: " + leftRearPower[1]);
            telemetry.update();

            robot.rightFront.setVelocity(rightFrontPower[0]);
            robot.rightRear.setVelocity(rightRearPower[0]);
            robot.leftFront.setVelocity(leftFrontPower[0]);
            robot.leftRear.setVelocity(leftRearPower[0]);
        }
    }

    /* ----------------------- PID FUNCTION ----------------------- */
    public double[] returnPower(int target, int current, double[] kvals) {
        double err = target - current;
        integralSum += err*timer.seconds();
        double derivative = (err-lastError) / timer.seconds();
        lastError = err;

        timer.reset();

        double[] d = {((err*kvals[0]) + (derivative * kvals[2]) + (integralSum * kvals[1])), lastError};
        return d;
    }
}
