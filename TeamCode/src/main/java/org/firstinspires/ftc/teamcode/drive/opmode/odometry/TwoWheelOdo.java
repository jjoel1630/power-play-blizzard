package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Config
@Autonomous(name="2 Wheel Odo", group="odometry")
public class TwoWheelOdo extends LinearOpMode {
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry = leftFront, leftRear, rightFront;
        robot.init(this, hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()) {
            while(robot.leftFront.getCurrentPosition() <= 1500) {
                robot.moveOnPower(0.7);
            }
            robot.quitBot();
        }
    }
}
