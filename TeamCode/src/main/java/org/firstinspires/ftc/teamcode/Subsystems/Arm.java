package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    private DcMotorEx shoulderMotor;
    private DcMotorEx elbowMotor;
    private double motor_power = 0.2;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.shoulderMotor = hwMap.get(DcMotorEx.class, "shoulderMotor" );
        this.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.elbowMotor = hwMap.get(DcMotorEx.class, "elbowMotor" );
        this.elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void mooveShoulder(boolean up, boolean down){
       if (up){
           shoulderMotor.setPower(motor_power);
        }
       else if (down) {
           shoulderMotor.setPower(-motor_power);

       }
    }

    public void mooveArm (boolean up, boolean down){
        if (up){
            elbowMotor.setPower(motor_power);
        } else if (down) {
            elbowMotor.setPower(-motor_power);

        }
    }
}