package org.firstinspires.ftc.teamcode.drive.opmode.teleopm4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Config
@TeleOp(name="Teleop Auton Test 1", group="Teleop Auton")
public class TeleOpAuton extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // Target A
    Vector2d targetAVector = new Vector2d(45, 45);
    double targetAHeading = Math.toRadians(90);

    // Target B
    Vector2d targetBVector = new Vector2d(-15, 25);
    double targetAngle = Math.toRadians(45);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
    }
}
