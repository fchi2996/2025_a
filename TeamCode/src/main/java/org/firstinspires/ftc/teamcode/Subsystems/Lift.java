package org.firstinspires.ftc.teamcode.Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    private DcMotorEx liftMotor;
    private

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.liftMotor = hwMap.get(DcMotorEx.class, "liftMotor" );
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Ascend(double up, double down){
        double direction = 0.0;
        direction = up - down;
        if (direction != 0.0) {
            liftMotor.setPower(direction);
        }
        else{
            liftMotor.setPower(0.0);
        }
    }
}
