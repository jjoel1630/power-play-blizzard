package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdoPositioning implements Runnable {
    // Wheels
    public DcMotor left, right, norm;

    // Position vars
    public double curLeftPos = 0, curRightPos = 0, curNormPos = 0, headingChange = 0;
    public double prevLeftPos = 0, prevRightPos = 0, prevNormPos = 0;
    public double roboX = 0, roboY = 0, roboHeading = 0;

    // Encoder direction
    private int leftDir = 1;
    private int rightDir = 1;
    private int normDir = 1;

    // Algo Constants
    public double encoderWheelDist;
    public double horizontalEncoderTicksPerDegreeOffset;
    public long sleeptime;

    public boolean isRunning = true;

    public OdoPositioning(DcMotor left, DcMotor right, DcMotor norm, double countsPerIn) {
        this.left = left;
        this.right = right;
        this.norm = norm;

        encoderWheelDist = 0 * countsPerIn;
        horizontalEncoderTicksPerDegreeOffset = 0;
    }

    public void updatePos() {
        // Cur positions
        curLeftPos = left.getCurrentPosition() * leftDir;
        curRightPos = right.getCurrentPosition() * rightDir;

        double leftChange = curLeftPos - prevLeftPos;
        double rightChange = curRightPos - prevRightPos;

        // Heading
        headingChange = (leftChange - rightChange) / encoderWheelDist;
        roboHeading = headingChange + headingChange;

        curNormPos = norm.getCurrentPosition() * normDir;
        double rawNormChange = curNormPos - prevNormPos;
        double normChange = rawNormChange - (headingChange * horizontalEncoderTicksPerDegreeOffset);

        double p = (leftChange + rightChange) / 2.0;
        double n = normChange;

        roboX = roboX + (p*Math.sin(roboHeading) + n*Math.cos(roboHeading));
        roboY = roboY + (p*Math.cos(roboHeading) + n*Math.sin(roboHeading));

        prevLeftPos = curLeftPos;
        prevRightPos = curRightPos;
        prevNormPos = curNormPos;
    }

    public double getXCoord() { return roboX; }

    public double getYCoord() { return roboY; }

    public double getHeading() { return Math.toDegrees(roboHeading) % 360; }

    public void stop() { isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            updatePos();
            try {
                Thread.sleep(sleeptime);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
