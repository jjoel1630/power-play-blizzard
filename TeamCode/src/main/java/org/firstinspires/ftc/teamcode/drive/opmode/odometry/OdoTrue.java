package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdoTrue {
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderNormal;

    public static double lateral = 20.12;
    public static double midpoint = 11.5;
    public static double wheelRad = 3.0;
    public static double cmTick = 2.0 * Math.PI * wheelRad / 8192;

    public int curRightPos = 0;
    public int curLeftPos = 0;
    public int curNormPos = 0;

    public int prevRightPos = 0;
    public int prevLeftPos = 0;
    public int prevNormPos = 0;

    public XyhVector start = new XyhVector(213, 102, Math.toRadians(90));
    public XyhVector pos = new XyhVector(start);

    private HardwareMap hMap;

    public OdoTrue(HardwareMap hM) {
        hMap = hM;

        DcMotor left = hMap.dcMotor.get("left");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor right = hMap.dcMotor.get("right");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor normal = hMap.dcMotor.get("normal");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderLeft = left;
        encoderRight = right;
        encoderNormal = normal;
    }

    public void odometry() {
        prevRightPos = curRightPos;
        prevLeftPos = curLeftPos;
        prevNormPos = curNormPos;

        curRightPos = -encoderRight.getCurrentPosition();
        curLeftPos = encoderLeft.getCurrentPosition();
        curNormPos = encoderNormal.getCurrentPosition();

        int dn1 = curLeftPos  - prevLeftPos;
        int dn2 = curRightPos - prevRightPos;
        int dn3 = curNormPos - prevNormPos;

        // the robot has moved and turned a tiny bit between two measurements:
        double dtheta = cmTick * ((dn2-dn1) / (lateral));
        double dx = cmTick * ((dn1+dn2) / 2.0);
        double dy = cmTick * (dn3 + ((dn2-dn1) / 2.0));

        // small movement of the robot gets added to the field coordinate system:
        pos.heading += dtheta / 2;
        pos.x += dx * Math.cos(pos.heading) - dy * Math.sin(pos.heading);
        pos.y += dx * Math.sin(pos.heading) + dy * Math.cos(pos.heading);
    }
}
