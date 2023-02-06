package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Disabled
@Config
@Autonomous(name="Odo Naked Drivetrain", group="odometry")
public class OdoNakedDt extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();

    DcMotorEx left, right, norm;

    public static double rad = 1.88976;
    public static double tpr = 8192;
    public static double inprev = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry = leftFront, leftRear, rightFront;
        left = hardwareMap.get(DcMotorEx.class, "leftFront");
        norm = hardwareMap.get(DcMotorEx.class, "rightRear");
        right = hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();

        timer.reset();

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        norm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive()) {
            int lpos = left.getCurrentPosition();
            int rpos = right.getCurrentPosition();
            int npos = norm.getCurrentPosition();

            double circum = (Math.PI * 2 * rad);

            double lradPos = Math.round(((lpos / tpr) * circum) * 100.0) / 100.0;
            double rradPos = Math.round(((rpos / tpr) * circum) * 100.0) / 100.0;
            double nradPos = Math.round(((npos / tpr) * circum) * 100.0) / 100.0;

            double lsimPos = Math.round(((lpos / tpr) * inprev) * 100.0) / 100.0;
            double rsimPos = Math.round(((rpos / tpr) * inprev) * 100.0) / 100.0;
            double nsimPos = Math.round(((npos / tpr) * inprev) * 100.0) / 100.0;


            telemetry.addLine("Left (ticks): " + lpos);
            telemetry.addLine("Right (ticks): " + rpos);
            telemetry.addLine("Norm (ticks): " + npos);

            telemetry.addLine("Radius-based pos Left (in): " + lradPos);
            telemetry.addLine("Radius-based pos Right (in): " + rradPos);
            telemetry.addLine("Radius-based pos Norm (in): " + nradPos);

            telemetry.addLine("Simulated pos Left (in): " + lsimPos);
            telemetry.addLine("Simulated pos Right (in): " + rsimPos);
            telemetry.addLine("Simulated pos Normal (in): " + nsimPos);

            telemetry.addLine("Circumference (in): " + circum);

            telemetry.update();

            timer.reset();
        }
    }
}
