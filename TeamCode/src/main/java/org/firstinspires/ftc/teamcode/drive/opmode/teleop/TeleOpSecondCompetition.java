package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Config
@TeleOp(name="TeleOp 2nd Comp", group="Linear OpMode")
public class TeleOpSecondCompetition extends LinearOpMode {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotorEx linearSlideMotor = null;

    private Servo claw;

    private double CLAW_MIN= 0.0;
    private double CLAW_MAX = 0.8;
    private double CLAW_HOME = CLAW_MIN;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean linearIsZero = false;

        claw = hardwareMap.servo.get("claw");
//        claw.setDirection(Servo.Direction.REVERSE);

        double clawPos = CLAW_HOME;
        double clawSpeed = 0.08;

        waitForStart();

        while (opModeIsActive()) {
            double max;
            double axial   = -gamepad1.left_stick_y;  // forward, back
            double lateral = gamepad1.right_stick_x; // turning
            double yaw     =  gamepad1.left_stick_x; // side to side

            /* ---------------------- CLAW MOVEMENT ---------------------- */
            if(gamepad2.b) claw.setPosition(CLAW_HOME);
            if(gamepad2.right_trigger == 1.0) clawPos = CLAW_MAX;
            else if(gamepad2.left_trigger == 1.0) clawPos = CLAW_MIN;

            clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
            claw.setPosition(clawPos);

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
            telemetry.addData("Slide pos", linearSlideMotor.getCurrentPosition());
            telemetry.update();

            if(gamepad2.y) {
                linearSlideMotor.setTargetPosition(-1100);
                linearSlideMotor.setPower(0.7);
                linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while(linearSlideMotor.getCurrentPosition() <= 2260) {
//                    telemetry.addLine("position: " + linearSlideMotor.getCurrentPosition());
//                    telemetry.update();
//                }
            } else {
                linearSlideMotor.setPower(linearPower);
            }
        }
    }
}
