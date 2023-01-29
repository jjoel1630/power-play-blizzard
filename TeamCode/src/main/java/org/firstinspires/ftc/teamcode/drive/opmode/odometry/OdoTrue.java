package org.firstinspires.ftc.teamcode.drive.opmode.odometry;

import android.text.method.TextKeyListener;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdoTrue {
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderNormal;

    public double lateral;
    public double midpoint;
    public double wheelRad;
    public double inTick;

    public boolean stopCounting = false;

    public int curRightPos = 0;
    public int curLeftPos = 0;
    public int curNormPos = 0;

    public int prevRightPos = 0;
    public int prevLeftPos = 0;
    public int prevNormPos = 0;

    public XyhVector start = new XyhVector(0, 0, Math.toRadians(0));
    public XyhVector pos = new XyhVector(start);

    private HardwareMap hMap;

    public OdoTrue(HardwareMap hM, double wheelRad, double lateral, double midpoint) {
        hMap = hM;

        DcMotor left = hMap.dcMotor.get("leftFront");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotor.Direction.REVERSE);

        DcMotor right = hMap.dcMotor.get("rightFront");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotor.Direction.REVERSE);

        DcMotor normal = hMap.dcMotor.get("leftRear");
        normal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        normal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        normal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderLeft = left;
        encoderRight = right;
        encoderNormal = normal;

        this.lateral = lateral;
        this.wheelRad = wheelRad;
        this.midpoint = midpoint;
        this.inTick = (2.0 * Math.PI * wheelRad) / 8192;
    }

    public OdoTrue(HardwareMap hM) {
        hMap = hM;

        DcMotor left = hMap.dcMotor.get("leftFront");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor right = hMap.dcMotor.get("rightFront");
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor normal = hMap.dcMotor.get("leftRear");
        normal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        normal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        normal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderLeft = left;
        encoderRight = right;
        encoderNormal = normal;
    }

    public void odometry(Telemetry t) {
        prevRightPos = curRightPos;
        prevLeftPos = curLeftPos;
        prevNormPos = curNormPos;

        curRightPos = -encoderRight.getCurrentPosition();
        curLeftPos = encoderLeft.getCurrentPosition();
        curNormPos = encoderNormal.getCurrentPosition();

//        t.addLine("curRight: " + curRightPos + ", curLeft: " + curLeftPos + ", curNorm" + curNormPos);
//        t.addLine("prevRightPos: " + prevRightPos + ", prevLeftPos: " + prevLeftPos + ", prevNormPos" + prevNormPos);
//        t.addLine("intick: " + inTick);

        int dn1 = curLeftPos - prevLeftPos;
        int dn2 = curRightPos - prevRightPos;
        int dn3 = curNormPos - prevNormPos;

//        t.addLine("dn1: " + dn1 + ", dn2: " + dn2 + ", dn3: " + dn3);

        // the robot has moved and turned a tiny bit between two measurements:
        double dtheta = inTick * ((dn2-dn1) / lateral);
        double dx = inTick * ((dn1+dn2) / 2.0);
        double dy = inTick * (dn3 - ((dn2-dn1) * (midpoint / lateral)));

//        t.addLine("dtheta: " + dtheta + ", dx: " + dx + ", dy: " + dy);

        // small movement of the robot gets added to the field coordinate system:
        pos.heading += dtheta;
        pos.x += dx * Math.cos(pos.heading) - dy * Math.sin(pos.heading);
        pos.y += dx * Math.sin(pos.heading) + dy * Math.cos(pos.heading);

        double x = Math.round(pos.x * 100.0) / 100.0;
        double y = Math.round(pos.y * 100.0) / 100.0;
        double dt = Math.round(dtheta * 100.0) / 100.0;
        double h = Math.round((Math.toDegrees(pos.heading)%360) * 100.0) / 100.0;
        double hrad = Math.round((pos.heading%(2*Math.PI)) * 100.0) / 100.0;

        if(hrad < 0 || h < 0) {
            hrad += (2*Math.PI);
            h += 360;
        }

        t.addLine("dtheta: " + dt + ", heading rad: " + hrad + ", heading deg: " + h);
        t.addLine("(" + x + ", " + y + ")");
//        t.addLine("X pos, Y pos, Heading: " + pos.x + " " + pos.y + " " + pos.heading);
//        t.addLine("X pos, Y pos, Heading: " + pos.x + " " + pos.y + " " + pos.heading);
        t.update();
    }
}
