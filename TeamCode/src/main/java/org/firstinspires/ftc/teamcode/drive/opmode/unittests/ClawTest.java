package org.firstinspires.ftc.teamcode.drive.opmode.unittests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Claw Testing", group="Neat Unit Testing")
public class ClawTest extends LinearOpMode {
    public static double clawPos = 0.0;
    public static double CLAW_MAX = 1.0;
    public static double CLAW_MIN = 0.0;

    public Servo claw;
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ------------------------- IMPORTANT READ FIRST!!! -------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------- */

    // HOW TO ZERO A MOTOR
    // 1. FIRST DETERMINE WHICH DIRECTION YOU WANT YOUR COMPONENT TO MOVE. THEN ***********REMOVE THE COMPONENT**********
    // 2. THE SERVO WILL MOVE IN 2 DIRECTIONS
    // 2i. IT WILL EITHER MOVE IN THE CLOCKWISE DIRECTION TO ITS MAXIMUM POSITION AND THEN COUNTERCLOCKWISE TO MINIMUM
    // 2ii. OR IT WILL MOVE IN THE COUNTERCLOCKWISE DIRECTION TO ITS MAXIMUM POSITION AND THEN THE CLOCKWISE POSITION TO MINIMUM
    // 3. YOU NEED TO FIGURE OUT WHICH ONE IT IS BY USING THE A AND Y KEYS TO MOVE THE SERVO BACK AND FORTH
    // 4. THEN, YOU NEED TO USE THIS INFO TO INSTALL THE CLAW AT EITHER ITS MAXIMUM OR MINIMUM POSITION

    // NOTE: THE MIN AND MAX *****DOES NOT MATTER*****. PLEASE DON'T SAY IS THIS MIN OR MAX, JUST LOOK AT THE DIRECTION OF THE SPIN
    // IT ONLY MATTERS WHAT DIRECTION IT SPINS IN!!!!!!!!!!!!!

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.servo.get("claw");

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad2.right_trigger == 1.0) clawPos = CLAW_MAX;
            else if(gamepad2.left_trigger == 1.0) clawPos = CLAW_MIN;

            clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
            claw.setPosition(clawPos);
        }
    }
}
