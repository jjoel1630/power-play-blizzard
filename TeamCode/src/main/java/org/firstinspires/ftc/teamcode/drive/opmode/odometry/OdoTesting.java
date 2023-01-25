package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.bot.PowerPlayBot;

@Config
@Autonomous(name="Odo Testing", group="odometry")
public class OdoTesting extends LinearOpMode {
    PowerPlayBot robot = new PowerPlayBot();
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry = leftFront, leftRear, rightFront;
        robot.init(this, hardwareMap, telemetry);

        OdoTrue drive = new OdoTrue(hardwareMap);

        waitForStart();

        timer.reset();

        while(opModeIsActive()) {
            drive.odometry();

            telemetry.addData("Left, Right, Normal", "%6d      %6d      %6d", drive.curLeftPos, drive.curRightPos, drive.curNormPos);
            telemetry.addData("X pos, Y pos, Heading", "%6.1f cm    %6.1f cm   %6.1f cm", drive.pos.x, drive.pos.y, drive.pos.heading);
            telemetry.update();
            timer.reset();
        }
    }
}
