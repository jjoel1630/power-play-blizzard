package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

public class XyhVector {
    public double x;
    public double y;
    public double heading;

    public XyhVector(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public XyhVector(XyhVector v) {
        this.x = v.x;
        this.y = v.y;
        this.heading = v.heading;
    }
}
