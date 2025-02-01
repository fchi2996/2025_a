package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hand {
    private Servo wristServo;
    private Servo fingerServo;



    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.fingerServo = hwMap.get(Servo.class, "fingerServo");
        this.wristServo = hwMap.get(Servo.class, "wristServo");

    }
}
