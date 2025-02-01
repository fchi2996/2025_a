package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    private DcMotorEx shoulderMotor;
    private DcMotorEx elbowMotor;
    public static double motor_power = 0.2;
    private PIDController shoulder_controller;
    public static double shoulder_kP;
    public static double shoulder_kI;
    public static double shoulder_kD;



    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.shoulderMotor = hwMap.get(DcMotorEx.class, "shoulderMotor" );
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.shoulder_controller = new PIDController (shoulder_kP, shoulder_kI, shoulder_kD);

        this.elbowMotor = hwMap.get(DcMotorEx.class, "elbowMotor" );
        this.elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void mooveShoulder(boolean upp, boolean downn){
       if (upp){
           shoulderMotor.setPower(motor_power);
        }
       else if (downn) {
           shoulderMotor.setPower(-motor_power);
       }
       else {
           shoulderMotor.setPower(0.0);
       }

    }

    public void mooveArm (boolean up, boolean down){

        dashboardTelemetry.addData("befor up values", up);

        if (up){
            dashboardTelemetry.addData("up values",up);
            dashboardTelemetry.update();
            elbowMotor.setPower(motor_power);
        }
        else if (down) {
            elbowMotor.setPower(-motor_power);
            dashboardTelemetry.addData("down values", down);
            dashboardTelemetry.update();
        }
        else {
            dashboardTelemetry.addData("up else", up);
            elbowMotor.setPower(0.0);

        }


    }
}