
package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp

public class DriveClass extends LinearOpMode

{

    // Declare OpMode members.

    //Plane Motor

    private DcMotor planeMotor = null;

    //Motor Speed

    private double turningSpeedMultiplier = 1;
    private double robotSpeedMultiplier = 2;
    private double wristSpeed = 250;

    //Wheel Motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple frontRight = null;
    private DcMotorSimple frontLeft = null;
    private DcMotorSimple backRight = null;
    private DcMotorSimple backLeft = null;


    //Arm Motors
    private DcMotorEx armLeft = null;
    private DcMotorEx armRight = null;

    private DcMotorEx wristMotor = null;
    private Servo fingersMotor = null;

    private int desiredPositionLeft = 0;
    private int desiredPositionRight = 0;


    //JoyStick references
    private double lJoyStickY = 0.0;
    private double lJoyStickX = 0.0;

    private double rJoyStickY = 0.0;
    private double rJoyStickX = 0.0;

    //Variables
    private double angle = 0.0;
    private double armVelocity = 1000;
    private double wrist_position = 0;
    private boolean fingersMode = false;

    //Sensors
    private DistanceSensor sensor;

    @Override
    public void runOpMode()
    {

        //---HardwareMap---

        telemetry.addData("Status", "Initialized");
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        //---Arm Hardware Map---

        armLeft = hardwareMap.get(DcMotorEx.class, "armLeft");
        armRight = hardwareMap.get(DcMotorEx.class, "armRight");
        wristMotor = hardwareMap.get(DcMotorEx.class, "wristMotor");
        fingersMotor = hardwareMap.get(Servo.class, "fingersMotor");

        //--- Plane Hardware Map ---

        planeMotor = hardwareMap.get(DcMotor.class, "launcher");

        //---Arm Set Modes---

        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fingersMotor.setPosition(0.8);

        //--- Plane Set Modes ---

        planeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        planeMotor.setPower(1);

        //--------------
        waitForStart();
        runtime.reset();

        //telemetry.update();

        while (opModeIsActive())
        {

            telemetry.addData("Left MotorBehaviour: ", armLeft.getZeroPowerBehavior());
            telemetry.addData("Right MotorBehaviour: ", armRight.getZeroPowerBehavior());
            telemetry.addData("Left Position: ", armLeft.getCurrentPosition());
            telemetry.addData("Right Position: ", armRight.getCurrentPosition());

            telemetry.addData("Left Target Position: ", armLeft.getTargetPosition());
            telemetry.addData("Fingers position: ", fingersMotor.getPosition());
            telemetry.addData("left Trigger: ", gamepad2.left_trigger);
            telemetry.addData("Wrist Motor Position: ", wristMotor.getCurrentPosition());
            telemetry.addData("Plane Motor Position: ", planeMotor.getCurrentPosition());
            telemetry.update();

            JoyStickConversion();
            AngleConversion();

            frontLeft.setPower(-PowerSin() * robotSpeedMultiplier - rJoyStickX*turningSpeedMultiplier); // /1.5
            frontRight.setPower(-PowerCos() * robotSpeedMultiplier - rJoyStickX*turningSpeedMultiplier);
            backLeft.setPower(PowerCos() * robotSpeedMultiplier -  rJoyStickX*turningSpeedMultiplier);
            backRight.setPower(PowerSin() * robotSpeedMultiplier - rJoyStickX*turningSpeedMultiplier);

            //------Arm-----

            Arm();

            //------Fingers-----

            Fingers();

            //---------Wrist-------

            Wrist();

            //------Plane-------

            Plane();



        }
    }


    private double PowerSin()
    {
        double speed = Math.sqrt(lJoyStickX * lJoyStickX + lJoyStickY * lJoyStickY);
        return speed * Math.sin(Math.toRadians(angle) + Math.toRadians(45));

    }

    private double PowerCos()
    {

        double speed = Math.sqrt(lJoyStickX * lJoyStickX + lJoyStickY * lJoyStickY);
        return speed * Math.cos(Math.toRadians(angle) + Math.toRadians(45));

    }

    private void AngleConversion()
    {
        angle = Math.toDegrees(Math.atan2(lJoyStickY , lJoyStickX));
    }


    private void JoyStickConversion()
    {
        lJoyStickY = gamepad1.left_stick_y * -1;
        lJoyStickX = gamepad1.left_stick_x;

        rJoyStickY = gamepad1.right_stick_y * -1;
        rJoyStickX = gamepad1.right_stick_x;
    }

    private void Arm()
    {
        if (gamepad2.left_bumper)
        {

            armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armLeft.setVelocity(armVelocity);
            armRight.setVelocity(armVelocity * -1);
        }
        else if (gamepad2.right_bumper)
        {
            armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armLeft.setVelocity(armVelocity * -1);
            armRight.setVelocity(armVelocity);
        }
        else
        {
            armLeft.setVelocity(0);
            armRight.setVelocity(0);
        }
    }

    private void Fingers()
    {
        if (gamepad2.x)
        {
            if (fingersMode == false)
            {
                if (fingersMotor.getPosition() == 0.8) //closed
                {
                    fingersMotor.setPosition(0.5);
                }

                else if (fingersMotor.getPosition() == 0.5) //opened
                {
                    fingersMotor.setPosition(0.8);
                }
                fingersMode = true;
            }
        }

        else if (!gamepad2.x)
        {
            fingersMode = false;
        }
    }

    private void Wrist()
    {
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0)
        {
            wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wristMotor.setVelocity(wristSpeed);
        }
        if (gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0)
        {
            wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wristMotor.setVelocity(-wristSpeed);
        }
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)
        {
            wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wristMotor.setVelocity(0);
        }
    }

    private void Plane()
    {
        if (gamepad1.y)
        {
            planeMotor.setTargetPosition(62);
            planeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}






