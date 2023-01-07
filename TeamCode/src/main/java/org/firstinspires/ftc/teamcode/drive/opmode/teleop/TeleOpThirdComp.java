package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name="TeleOp 3rd Comp", group="Linear OpMode")
public class TeleOpThirdComp extends LinearOpMode {
    /* ---------------------- CONFIG VARS ---------------------- */
    public static int CLAW_REV = 0;
    public static int RBAR_REV = 0;
    public static int LBAR_REV = 0;

    public static double RBAR_MAX = 1.0;
    public static double LBAR_MAX = 1.0;
    public static double RBAR_MIN = 0.0;
    public static double LBAR_MIN = 0.0;

    public static double rightBarPos = RBAR_MAX;
    public static double leftBarPos = LBAR_MAX;

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotorEx linearSlideRightMotor = null;
    private DcMotorEx linearSlideLeftMotor = null;

    private Servo claw;
    private Servo rightBar;
    private Servo leftBar;

    private double CLAW_MIN= 0.0;
    private double CLAW_MAX = 0.8;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        linearSlideRightMotor = hardwareMap.get(DcMotorEx.class, "linearSlideRight");
        linearSlideRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlideLeftMotor = hardwareMap.get(DcMotorEx.class, "linearSlideLeft");
        linearSlideLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        boolean linearIsZero = false;

        claw = hardwareMap.servo.get("claw");
        rightBar = hardwareMap.servo.get("rightBar");
        leftBar = hardwareMap.servo.get("leftBar");

        if(CLAW_REV == 1) claw.setDirection(Servo.Direction.REVERSE);
        if(RBAR_REV == 1) rightBar.setDirection(Servo.Direction.REVERSE);
        if(LBAR_REV == 1) leftBar.setDirection(Servo.Direction.REVERSE);

        double clawPos = CLAW_MIN;
        double clawSpeed = 0.08;

        rightBarPos = Range.clip(rightBarPos, RBAR_MIN, RBAR_MAX);
        rightBar.setPosition(rightBarPos);

        leftBarPos = Range.clip(leftBarPos, LBAR_MIN, LBAR_MAX);
        leftBar.setPosition(leftBarPos);

        boolean stop = true;

        waitForStart();

        while (opModeIsActive()) {
            double max;
            double axial   = -gamepad1.left_stick_y;  // forward, back
            double lateral = gamepad1.right_stick_x; // turning
            double yaw     =  gamepad1.left_stick_x; // side to side

            /* ---------------------- CLAW MOVEMENT ---------------------- */
            if(gamepad2.b) claw.setPosition(CLAW_MIN);
            if(gamepad2.right_trigger == 1.0) clawPos = CLAW_MAX;
            else if(gamepad2.left_trigger == 1.0) clawPos = CLAW_MIN;

            clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
            claw.setPosition(clawPos);

            /* ---------------------- 2-BAR MOVEMENT ---------------------- */
//            double servoAxial = gamepad2.right_stick_y;
//            servoAxial = (servoAxial + 1.0) / 2.0;
//
//            rightBarPos = Range.clip(servoAxial, RBAR_MIN, RBAR_MAX);
//            rightBar.setPosition(rightBarPos);
//
//            leftBarPos = Range.clip(servoAxial, LBAR_MIN, LBAR_MAX);
//            leftBar.setPosition(leftBarPos);

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



            /* ---------------------- ROBOT MOVEMENT ---------------------- */
            double axialCoefficient = 1; // 0.53
            double yawCoefficient = 1; // 0.9
            double lateralCoefficient = 1; // 0.53
            yaw = yaw * yawCoefficient;
            axial = axial * axialCoefficient;
            lateral = lateral * lateralCoefficient;

            boolean slowModeOn = false;

            if(gamepad1.left_bumper) slowModeOn = true;
            if(gamepad1.right_bumper) slowModeOn = false;

            double powerModifier = slowModeOn ? 0.3 : 1;
            double leftFrontPower  = (axial + lateral + yaw) * powerModifier;
            double rightFrontPower = (axial - lateral - yaw) * powerModifier;
            double leftBackPower   = (axial - lateral + yaw) * powerModifier;
            double rightBackPower  = (axial + lateral - yaw) * powerModifier;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            /* ---------------------- LINEAR SLIDE MOVEMENT ---------------------- */
            double linearPowerModifier = 1.5;
            double linearAxial = gamepad2.left_stick_y;
            linearAxial = linearAxial * 0.5;
            double linearPower = linearAxial * linearPowerModifier;

//            if(linearSlideMotor.getCurrentPosition() >= 0) linearIsZero = true;
//            else linearIsZero = false;
//
//            if(linearIsZero) linearSlideMotor.setPower(0);
//            else linearSlideMotor.setPower(linearPower);

            telemetry.addData("Leftfront power", leftFrontPower);
            telemetry.addData("Leftback power", leftFrontPower);
            telemetry.addData("RightFront power", leftFrontPower);
            telemetry.addData("Rightback power", leftFrontPower);
            telemetry.addData("Left Slide pos", linearSlideLeftMotor.getCurrentPosition());
            telemetry.addData("Right Slide pos", linearSlideRightMotor.getCurrentPosition());
            telemetry.update();

            linearSlideLeftMotor.setPower(linearPower);
            linearSlideRightMotor.setPower(linearPower);
        }
    }
}
