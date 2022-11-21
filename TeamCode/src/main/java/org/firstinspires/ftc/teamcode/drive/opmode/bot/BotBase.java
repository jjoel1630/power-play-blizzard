package org.firstinspires.ftc.teamcode.drive.opmode.bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface BotBase {
    void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception;
}