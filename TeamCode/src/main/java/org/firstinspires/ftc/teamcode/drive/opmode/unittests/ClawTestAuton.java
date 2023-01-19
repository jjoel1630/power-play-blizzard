package org.firstinspires.ftc.teamcode.drive.opmode.unittests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Claw Testing Auton", group="Neat Unit Testing")
public class ClawTestAuton extends LinearOpMode {
    public static double clawPos = 0.0;
    public static double CLAW_MAX = 1.0;
    public static double CLAW_MIN = 0.0;
    public static double breakTime = 10.0; // break time between open and close (in seconds)

    public Servo claw;
    public ElapsedTime timer;

    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ------------------------- ZERO OUT THE SERVOS FIRST!!!!!! ------------------ */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */

    @Override
    public void runOpMode() throws InterruptedException {
        RobotLog.addGlobalWarningMessage("PLEASE LOOK AT THE CLAWTEST.JAVA FILE & ZERO OUT THE MOTORS BEFORE YOUR PROCEED WITH THIS OPMODE.");

        claw = hardwareMap.servo.get("claw");

        waitForStart();
        timer = new ElapsedTime();

        while(opModeIsActive()) {
            claw.setPosition(clawPos == CLAW_MIN ? CLAW_MAX : CLAW_MIN);

            timer.reset();
            while(timer.seconds() <= breakTime) {}
        }
    }
}
