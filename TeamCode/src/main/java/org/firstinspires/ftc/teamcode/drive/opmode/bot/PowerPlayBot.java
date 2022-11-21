package org.firstinspires.ftc.teamcode.drive.opmode.bot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import android.util.Log;

import java.util.ArrayList;

public class PowerPlayBot implements BotBase {
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    private HardwareMap hwMap = null;
    private Telemetry telemetry;

    private ElapsedTime runtime;

    private LinearOpMode owner = null;

    public static String LEFT_FRONT = "leftFront";
    public static String LEFT_REAR = "leftRear";
    public static String RIGHT_FRONT = "rightFront";
    public static String RIGHT_REAR = "rightRear";

    private int encoderDirection = 1;

    public PowerPlayBot() {};

    public void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception {
        this.owner = owner;
        this.hwMap = hw;
        this.telemetry = t;

        try {
            // Initialize Motors
            leftFront = hwMap.get(DcMotorEx.class, LEFT_FRONT);
            leftRear = hwMap.get(DcMotorEx.class, LEFT_REAR);
            rightFront = hwMap.get(DcMotorEx.class, RIGHT_FRONT);
            rightRear = hwMap.get(DcMotorEx.class, RIGHT_REAR);

            // Set direction of motors
            encoderDirection = 1;

            // Make sure no motors are null, and then set the direction + mode of each motor
            if (leftFront != null) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                rightRear.setDirection(DcMotor.Direction.FORWARD);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            quitBot();
        } catch (Exception ex) {
            Log.e("POWERPPLAYBOT", "Inititialization error: ", ex);
            throw new Exception("Error with ControlHub Config", ex);
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

    public boolean checkMotorTrue() {
        return (leftFront != null && leftRear != null && rightFront != null && rightRear != null);
    }
}
