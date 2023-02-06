package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@TeleOp(name="LinearSlide", group="Linear OpMode")
public class LinearSlide extends LinearOpMode {
    private DcMotorEx linearSlideMotor;
    private Servo claw;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
//        claw.setDirection(Servo.Direction.REVERSE);
//        claw.scaleRange(0.4, 0.6);
        // claw = hardwareMap.servo.get("claw");
        // claw = hardwareMap.get(Servo.class, "claw");
        // claw.setPosition(0);
//        double clawPosition = claw.getPosition();
//        claw.setPosition(0.0);

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean pressed = false;

        waitForStart();

        while (!isStopRequested()) {
//            telemetry.addLine("max: " + claw.MAX_POSITION + " - min: " + claw.MIN_POSITION + " - direction: " + claw.getDirection());
//            telemetry.update();

            double powerModifier = 1.0;
            double axial = gamepad1.left_stick_y;
//            double yaw     =  gamepad1.left_stick_x; // side to side
            axial = axial * 0.5;
//            yaw = yaw * 0.5;

            double linearPower  = axial * powerModifier;
//            double servoPower = (yaw * powerModifier);
            if(gamepad1.a && !pressed) {
                telemetry.addLine("claw: " + claw.getPosition());
                telemetry.update();
                claw.setPosition(0.8);
//                pressed = !pressed;
            }
            if(gamepad1.b && !pressed) {
                telemetry.addLine("claw: " + claw.getPosition());
                telemetry.update();
                claw.setPosition(0.2);
            }
            if(gamepad1.y) {
//                claw.setDirection(Servo.Direction.REVERSE);
                claw.setPosition(claw.getPosition());
            }

//            if(pressed && (gamepad1.a || gamepad1.b)) {
//                claw.setPosition(claw.getPosition());
//            }

            telemetry.addLine("claw: " + claw.getPosition());
            telemetry.update();
//            if(linearPower > 1.0) linearPower /= linearPower;

//            linearSlideMotor.getCurrentPosition() <= 1500

            linearSlideMotor.setPower(linearPower);

//          telemetry.addLine("claw: " + linearSlideMotor.getCurrentPosition());
//            telemetry.addLine("claw: " + yaw + " - power: " + servoPower);

//            telemetry.update();
        }
    }
}
