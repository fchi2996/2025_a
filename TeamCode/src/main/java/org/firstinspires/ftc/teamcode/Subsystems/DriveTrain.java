package org.firstinspires.ftc.teamcode.Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DriveTrain {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private double angle;
    private static double lStickX = 0.0;
    private static double lStickY = 0.0;
    private static double rStickX = 0.0;
    private static double rStickY = 0.0;
    private static double turningSpeedMultiplier = 1;
    private static double robotSpeedMultiplier = 2;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init (@NonNull HardwareMap hwMap){

        this.leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

     public void getController(double a, double b, double c, double d){
        lStickX = a;
        rStickX = b;
        lStickY = c;
        rStickY = d;
         dashboardTelemetry.addData("left Y",lStickY);
         dashboardTelemetry.addData("left X",lStickX);
         dashboardTelemetry.addData("right Y",rStickY);
         dashboardTelemetry.addData("right X",rStickX);
         //dashboardTelemetry.update();
     }
    public void Drive(){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = lStickY;  // Note: pushing stick forward gives negative value
        double lateral = lStickX;
        double yaw     = rStickX;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

        }
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }



}