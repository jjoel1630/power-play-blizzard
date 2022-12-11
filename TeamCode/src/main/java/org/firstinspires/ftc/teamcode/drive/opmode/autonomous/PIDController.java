package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double Kp = 0.0, Ki = 0.0, Kd = 0.0, Kf = 0.0;

    public double integralSum = 0.0;
    public double lastError = 0.0;

    ElapsedTime timer;


    public PIDController(double Kp, double Ki, double Kd, double Kf, ElapsedTime timer) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.timer = timer;
    }

    public double[] update(double target, double state, double time) {
        double err = target - state;
        integralSum += err * time;
        double derivative = (err - lastError) / time;
        lastError = err;

        double[] out = new double[3];

        double output = (err * Kp) + (derivative * Kd) + (integralSum * Ki);

        out[0] = output;
        out[1] = err;

        return out;
    }
}
