package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import android.content.Context;
import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

@Config
@Autonomous(name="odo 4x4", group="odometry")
public class OdoFourXFour extends LinearOpMode {
    public static int mat[][] = new int[4][4];

    public ArrayList<double[]> DATA = new ArrayList<double[]>();

    public int curX = 0;
    public int curY = 0;
    public int pcurX = 0;
    public int pcurY = 0;

    public static double lateral = 13;
    public static double midpoint = 6.5;
    public static double wheelRad = 1.88976;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry = leftFront = left, rightFront = right, leftRear = normal;
        NumTimes.times++;

        OdoTrue drive = new OdoTrue(hardwareMap, wheelRad, lateral, midpoint);

        waitForStart();

        timer.reset();
        double previousTime = timer.seconds();

//        while(opModeIsActive() && timer.seconds() <= 30) {
        while(opModeIsActive()) {
            drive.odometry(telemetry);

            double x = Math.round(drive.pos.x * 100.0) / 100.0;
            double y = Math.round(drive.pos.y * 100.0) / 100.0;
            double heading = Math.round((Math.toDegrees(drive.pos.heading) % 360) * 100.0) / 100.0;

//            curX = Range.clip((int)x / 24, 0, 3);
//            curY = Range.clip((int)y / 24, 0, 3);
//            mat[curX][curY] = 1;
//            mat[pcurX][pcurY] = 0;
//
//            pcurX = curX;
//            pcurY = curY;
//
//            String data = "";
//
//            for(int i = 0; i < mat.length; ++i) {
//                for(int j = 0; j < mat[i].length; ++j) {
//                    data += "    " + mat[i][j];
//                }
//                data += "\n";
//            }
//
//            telemetry.addLine("Left, Right, Normal: " + drive.curLeftPos + " " + drive.curRightPos + " " + drive.curNormPos);
//            telemetry.addLine("(" + x + ", " + y + ") | " + Math.toDegrees(heading));
//            telemetry.addLine(data);
//            telemetry.update();

//            double[] d = {x, y, heading, timer.seconds()};
//            DATA.add(d);
        }

//        try {
//            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(hardwareMap.appContext.openFileOutput("odoraw" + NumTimes.times, Context.MODE_PRIVATE));
////
////            for(double[] d : DATA) {
////                outputStreamWriter.write("X: " + d[0] + "\t Y: " + d[1] + "\t Heading: " + d[2] + "\t Timestamp: " + d[3] + "\n");
////            }
//            outputStreamWriter.write("Testing\n");
//
//            outputStreamWriter.close();
//        } catch (IOException e) {
//            telemetry.addData("Exception", "Configuration file write failed: " + e.toString());
//        }
//        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
//        File directory = new File(directoryPath);
//        //noinspection ResultOfMethodCallIgnored
//        directory.mkdir();
//        try {
//            FileWriter fileWriter = new FileWriter(directoryPath+"/test.txt");
//            fileWriter.write("test\n");
//            fileWriter.close();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
    }

    public boolean checkBounds(double max, double min, double cur) {
        if(cur >= max || cur <= min) return true;
        return false;
    }
}
