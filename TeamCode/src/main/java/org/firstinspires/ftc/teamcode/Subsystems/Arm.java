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
    public static double shoulder_kP = 0.01;
    public static double shoulder_kI = 0.028;
    public static double shoulder_kD = 0;

    private PIDController elbow_controller;
    public static double elbow_kP = 0.003;
    public static double elbow_kI = 0.05;
    public static double elbow_kD = 0;

    private static final double arm1Length = 15; // length of arm1 in cm
    private static final double arm2Length = 15; // length of arm2 in cm
    private static final double clawLength = 5; // length of claw in cm
    private static double arm2Trig = Math.sqrt(Math.pow(arm2Length,2) + Math.pow(clawLength,2));
    private double theta1 = 0;
    private double theta2 = 0;

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
        this.elbow_controller = new PIDController(elbow_kP, elbow_kI, elbow_kD);


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
    //pid controller output for shoulder motor
    public double shoulder_calc (double shoulder_pos, double shoulder_target){
        double output = 0.0;
        if (Math.abs(shoulder_target-shoulder_pos) > 5) {
            output = this.shoulder_controller.calculate(shoulder_pos, shoulder_target);
            output = PID_TEst.limiter(output, 1.0);

        }
        return output;
    }

    public double elbow_calc (double shoulder_pos, double shoulder_target){
        double output = 0.0;
        if (Math.abs(shoulder_target-shoulder_pos) > 5) {
            output = this.elbow_controller.calculate(shoulder_pos, shoulder_target);
            output = PID_TEst.limiter(output, 1.0);

        }
        return output;
    }

    // arm math angle
    public int armMath (double desiredLength, double desiredHeight){
        double q = Math.sqrt(Math.pow(desiredLength,2) + Math.pow(desiredHeight,2));
        theta1 = Math.PI - Math.atan(desiredHeight / desiredLength) - Math.acos( ( Math.pow(arm2Trig, 2) - Math.pow(arm1Length,2) - Math.pow(q,2)) / (-2 * arm1Length * q) );
        theta2 = Math.acos( (Math.pow(q,2) - Math.pow(arm1Length, 2) - Math.pow(arm2Length, 2)) / (-2 * arm1Length * arm2Length));
        return 0;
    }

}