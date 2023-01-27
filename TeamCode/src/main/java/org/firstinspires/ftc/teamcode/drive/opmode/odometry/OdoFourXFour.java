package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class OdoFourXFour extends LinearOpMode {
    public static int mat[][] = new int[4][4];

    public int curX = 0;
    public int curY = 0;
    public int pcurX = 0;
    public int pcurY = 0;

    public static double lateral = 20.12;
    public static double midpoint = 11.5;
    public static double wheelRad = 3.0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry = leftFront = left, rightFront = right, leftRear = normal;

        OdoTrue drive = new OdoTrue(hardwareMap, lateral, midpoint, wheelRad);

        waitForStart();

        timer.reset();

        while(opModeIsActive()) {
            drive.odometry();

            double x = drive.pos.x;
            double y = drive.pos.y;
            double heading = drive.pos.heading;

            curX = Range.clip((int)x / 24, 0, 3);
            curY = Range.clip((int)y / 24, 0, 3);
            mat[curX][curY] = 1;
            mat[pcurX][pcurY] = 0;

            pcurX = curX;
            pcurY = curY;

            String data = "";

            for(int i = 0; i < mat.length; ++i) {
                for(int j = 0; j < mat[i].length; ++j) {
                    data += "    " + mat[i][j];
                }
                data += "\n";
            }

            telemetry.addData("Left, Right, Normal", "%6d      %6d      %6d", drive.curLeftPos, drive.curRightPos, drive.curNormPos);
            telemetry.addData("X pos, Y pos, Heading", "%6.1f cm    %6.1f cm    %6.1f cm", drive.pos.x, drive.pos.y, drive.pos.heading);
            telemetry.addLine(data);
            telemetry.update();

            timer.reset();
        }
    }

    public boolean checkBounds(double max, double min, double cur) {
        if(cur >= max || cur <= min) return true;
        return false;
    }
}
