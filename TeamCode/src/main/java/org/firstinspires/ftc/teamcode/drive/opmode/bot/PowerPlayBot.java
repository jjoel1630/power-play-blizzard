package org.firstinspires.ftc.teamcode.drive.opmode.bot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import android.util.Log;

import java.util.ArrayList;

public class PowerPlayBot {
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    private HardwareMap hwMap = null;
    private Telemetry telemetry;

    private ElapsedTime runtime;

    private LinearOpMode owner = null;

    public static String LEFT_FRONT = "leftFront";
    public static String LEFT_REAR = "leftRear";
    public static String RIGHT_FRONT = "rightFront";
    public static String RIGHT_REAR = "rightRear";

    // Constants:
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0.7, 0, 5, 13);
    public static double WHEEL_RADIUS = 1.9685; // in
    public double circumference = WHEEL_RADIUS * 2 * Math.PI;
    public static double GEAR_RATIO = 1.223047;
    public static double TRACK_WIDTH = 14; // in
    public static double kV = 1.0 / (MAX_RPM * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0);
    public static double kA = 0;
    public static double kStatic = 0;
    public static double MAX_VEL = 76.22057483314033;
    public static double MAX_ACCEL = 76.22057483314033;
    public static double MAX_ANG_VEL = Math.toRadians(311.93694642857145);
    public static double MAX_ANG_ACCEL = Math.toRadians(311.93694642857145);

    private int encoderDirection = 1;

    private BNO055IMU imu;

    public PowerPlayBot() {};

    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) {
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;

        try {
            // Initialize Motors
            leftFront = hw.get(DcMotorEx.class, LEFT_FRONT);
            leftRear = hw.get(DcMotorEx.class, LEFT_REAR);
            rightFront = hw.get(DcMotorEx.class, RIGHT_FRONT);
            rightRear = hw.get(DcMotorEx.class, RIGHT_REAR);

            // Set direction of motors
            encoderDirection = 1;

            // Make sure no motors are null, and then set the direction + mode of each motor
            if (leftFront != null) {
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            }

            if (leftRear != null) {
                leftRear.setDirection(DcMotor.Direction.FORWARD);
                leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (rightFront != null) {
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (rightRear != null) {
                rightRear.setDirection(DcMotor.Direction.REVERSE);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRear.setDirection(DcMotorEx.Direction.REVERSE);
            }

            double rfF = 32767 / 2260;
            double rfP = 0.1 * rfF;
            double rfI = 0.1 * rfP;

            double rrF = 32767 / 2160;
            double rrP = 0.1 * rrF;
            double rrI = 0.1 * rrP;

            double lfF = 32767 / 2260;
            double lfP = 0.1 * lfF;
            double lfI = 0.1 * lfP;

            double lrF = 32767 / 2300;
            double lrP = 0.1 * lrF;
            double lrI = 0.1 * lrP;

//            rightFront.setVelocityPIDFCoefficients(rfP, rfI, 0, rfF);
//            rightRear.setVelocityPIDFCoefficients(rrP, rrI, 0, rrF);
//            leftFront.setVelocityPIDFCoefficients(lfP, lfI, 0, lfF);
//            leftRear.setVelocityPIDFCoefficients(lrP, lrI, 0, lrF);
//
//            rightFront.setPositionPIDFCoefficients(5.0);
//            rightRear.setPositionPIDFCoefficients(5.0);
//            leftFront.setPositionPIDFCoefficients(5.0);
//            leftRear.setPositionPIDFCoefficients(5.0);

            imu = hw.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            quitBot();
        } catch (Exception ex) {
            Log.e("POWERPPLAYBOT", "Inititialization error: ", ex);
//            throw new Exception("Error with ControlHub Config", ex);
        }
    }

    public void quitBot() {
        if(!checkMotorTrue()) return;

        // Set all motor powers to 0
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Set mode to run with encoder
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPositionMode() {
        leftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWOEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getVelocity(String name) {
        if(name == LEFT_FRONT) return leftFront.getVelocity();
        if(name == LEFT_REAR) return leftFront.getVelocity();
        if(name == RIGHT_FRONT) return leftFront.getVelocity();
        return rightRear.getVelocity();
    }

    public void moveMax() {
        if(!checkMotorTrue()) return;

        leftFront.setPower(1);
        leftRear.setPower(1);
        rightFront.setPower(1);
        rightRear.setPower(1);
    }

    public void moveAuto(double power) {
        if (checkMotorTrue()) {
//            double rightPower = Range.clip(power, -1.0, 1.0);
//            double leftPower = Range.clip(power, -1.0, 1.0);

            leftFront.setVelocity(0.01*power*1);
            rightFront.setVelocity(0.01*power*1);
            leftRear.setVelocity(0.01*power*1);
            rightRear.setVelocity(0.01*power*1);
        }
    }

    public void moveOnVelo(double velo) {
        if (checkMotorTrue()) {
//            double rightPower = Range.clip(power, -1.0, 1.0);
//            double leftPower = Range.clip(power, -1.0, 1.0);

            leftFront.setVelocity(velo);
            rightFront.setVelocity(velo);
            leftRear.setVelocity(velo);
            rightRear.setVelocity(velo);
        }
    }

    public void moveOnPower(double power) {
        if(!checkMotorTrue()) return;

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    public void strafeRight(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
    }

    public void strafeLeft(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }

    public void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
    }

    public void turnLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }

    public void setMotorTargetPos(int pos) {
        leftFront.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        leftRear.setTargetPosition(pos);
        rightRear.setTargetPosition(pos);
    }

    public void moveToPosition(double dist, double power) {
        resetEncoder();
        double rotationsNeeded = dist / circumference;
        int encoderDist = (int)(rotationsNeeded * TICKS_PER_REV);

        setMotorTargetPos(encoderDist);
        moveOnPower(power);
        runToPositionMode();

        while(isBusy()) {
            telemetry.addLine("driving " + encoderDist + " inches");
            telemetry.addLine("rightFront " + rightFront.getCurrentPosition() + " ticks");
            telemetry.addLine("rightRear " + rightRear.getCurrentPosition() + " ticks");
            telemetry.addLine("leftFront " + leftFront.getCurrentPosition() + " ticks");
            telemetry.addLine("leftRear " + leftRear.getCurrentPosition() + " ticks");
            telemetry.update();
        }

        quitBot();
    }

    public double getPosition() {
        return rightFront.getCurrentPosition();
    }

    public boolean isBusy() {
        return (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy());
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public boolean checkMotorTrue() {
        return (leftFront != null && leftRear != null && rightFront != null && rightRear != null);
    }
}
