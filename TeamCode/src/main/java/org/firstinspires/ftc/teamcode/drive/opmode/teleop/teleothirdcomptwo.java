package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="TeleOp 3rd Comp Part 2", group="Linear OpMode")
public class teleothirdcomptwo extends LinearOpMode {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    ElapsedTime timer;

    public static int encoderTicksHigh = 20;
    public static int encoderTicksMedium = 20;
    public static int encoderTicksLow = 20;

    public static double minDtPower = 0.4;
    public static double maxDtPower = 0.7;

    public static double linearPowerTesting = 0.05;

    private DcMotorEx linearSlide = null;

    private Servo claw;

    public static double CLAW_MIN = 0.75;
    public static double CLAW_MAX = 1.0;

    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

//        rightRear, leftRear, leftFront, rightFront
//        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
//        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
//        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.servo.get("claw");

        double clawPos = CLAW_MIN;
        boolean isDown = true;

        waitForStart();

        timer = new ElapsedTime();

        while (opModeIsActive()) {
            double max;
            double axial = -gamepad1.left_stick_y;  // forward, back
            double lateral = gamepad1.left_stick_x; // side to side
            double yaw = gamepad1.right_stick_x; // turning

            /* ---------------------- CLAW MOVEMENT ---------------------- */
            if(gamepad2.right_trigger == 1.0) clawPos = CLAW_MIN;
            else if(gamepad2.left_trigger == 1.0) clawPos = CLAW_MAX;

            clawPos = Range.clip(clawPos, CLAW_MIN, CLAW_MAX);
            claw.setPosition(clawPos);

            if(clawPos == CLAW_MIN && distanceSensor.getDistance(DistanceUnit.INCH) <= 1.0) {
                clawPos = CLAW_MAX;
                claw.setPosition(clawPos);
            }

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

            double powerModifier = slowModeOn ? 0.3 : 0.7;
            double leftFrontPower  = (axial + lateral + yaw) * powerModifier;
            double rightFrontPower = (axial - lateral - yaw) * powerModifier;
            double leftBackPower   = (axial - lateral + yaw) * powerModifier;
            double rightBackPower  = (axial + lateral - yaw) * powerModifier;

            if(gamepad1.right_trigger == 1.0) {
                timer.reset();
            }

            if(gamepad1.y) {
//                powerModifier = powerModifier * (timer.seconds() / 10);
//                powerModifier = Range.clip(powerModifier, -1 * minDtPower, maxDtPower);

                leftFrontPower  = powerModifier;
                rightFrontPower = powerModifier;
                leftBackPower   = powerModifier;
                rightBackPower  = powerModifier;
            }

            if(gamepad1.a) {
                leftFrontPower  = -powerModifier;
                rightFrontPower = -powerModifier;
                leftBackPower   = -powerModifier;
                rightBackPower  = -powerModifier;
            }

            if(gamepad1.b) {
                leftFrontPower  = powerModifier;
                rightFrontPower = -powerModifier;
                leftBackPower   = -powerModifier;
                rightBackPower  = powerModifier;
            }

            if(gamepad1.x) {
                leftFrontPower  = -powerModifier;
                rightFrontPower = powerModifier;
                leftBackPower   = powerModifier;
                rightBackPower  = -powerModifier;
            }

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
            boolean linearSlowModeOn = false;

            if(gamepad2.left_bumper) linearSlowModeOn = true;

            if(!linearSlide.isBusy()) linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double linearAxial = gamepad2.left_stick_y;
            linearAxial = linearAxial * 1;

            double linearPowerModifier = linearSlowModeOn ? 0.1 : 0.75;
//            double linearPowerModifier = linearPowerTesting;
            double linearPower = linearAxial * linearPowerModifier;

            if(linearSlide.getCurrentPosition() <= 10 && isDown) {
                linearPower = 0;
            }

            if(gamepad2.x) {
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide.setPower(linearPowerTesting);
                linearSlide.setTargetPosition(encoderTicksHigh);
            }

            if(gamepad2.y) {
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide.setPower(linearPowerTesting);
                linearSlide.setTargetPosition(encoderTicksMedium);
            }

            if(gamepad2.a) {
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlide.setPower(linearPowerTesting);
                linearSlide.setTargetPosition(encoderTicksLow);
            }

            if(gamepad2.b) {
                isDown = false;
            }

//            if(gamepad2.right_bumper) {
//                isDown = true;
//            }

            telemetry.addData("Leftfront power", leftFrontPower);
            telemetry.addData("Leftback power", leftBackPower);
            telemetry.addData("RightFront power", rightFrontPower);
            telemetry.addData("Rightback power", rightBackPower);
            telemetry.addData("range", String.format("%.01f inch", distanceSensor.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Slide pos", linearSlide.getCurrentPosition());
            telemetry.addData("Slide power", linearPower);
            telemetry.addData("Claw pos", claw.getPosition());
            telemetry.update();

            linearSlide.setPower(linearPower);
        }
    }
}
