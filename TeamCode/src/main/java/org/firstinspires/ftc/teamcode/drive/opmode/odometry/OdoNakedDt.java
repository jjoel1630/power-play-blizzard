package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="Odo Naked Drivetrain", group="odometry")
public class OdoNakedDt extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();

    DcMotorEx left, right, norm;

    public static double rad = 3.0;
    public static double tpr = 8192;
    public static double inprev = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry = leftFront, leftRear, rightFront;

        waitForStart();

        timer.reset();

        while(opModeIsActive()) {
            int lpos = left.getCurrentPosition();
            int rpos = right.getCurrentPosition();
            int npos = norm.getCurrentPosition();

            double lradPos = (lpos / tpr) * (Math.PI * 2 * rad);
            double rradPos = (lpos / tpr) * (Math.PI * 2 * rad);
            double nradPos = (lpos / tpr) * (Math.PI * 2 * rad);

            double lsimPos = (lpos / tpr) * inprev;
            double rsimPos = (lpos / tpr) * inprev;
            double nsimPos = (lpos / tpr) * inprev;

            telemetry.addLine("Left: " + lpos);
            telemetry.addLine("Right: " + rpos);
            telemetry.addLine("Norm: " + npos);

            telemetry.addLine("Radius-based pos Left: " + lradPos);
            telemetry.addLine("Radius-based pos Right: " + rradPos);
            telemetry.addLine("Radius-based pos Norm: " + nradPos);

            telemetry.addLine("Simulated pos Left: " + lsimPos);
            telemetry.addLine("Simulated pos Right: " + rsimPos);
            telemetry.addLine("Simulated pos Norm: " + nsimPos);

            telemetry.update();

            timer.reset();
        }
    }
}
