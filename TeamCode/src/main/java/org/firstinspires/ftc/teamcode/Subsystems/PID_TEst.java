package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


//this line identifies if this is an autonomous or teleop program
@Autonomous(name = "PID TEst")
//this line allows you to modify variables inside the dashboard
@Config
public class PID_TEst extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motor0;
    private Servo servo0;

    public static double current_position;
    public static double target_position = 500;
    public static double kI = 0;
    public static double kP = 0;
    public static double kD = 0;
    public PIDController pid;
    public static double maxPower = .1;


    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.motor0 = hardwareMap.get(DcMotorEx.class,"shoulderMotor");
        this.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.servo0 = hardwareMap.get(Servo.class, "fingerServo");

        pid = new PIDController(kP, kI, kD);

        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("power", motor0.getPower());
        dashboardTelemetry.addData("current position", motor0.getCurrentPosition());
        dashboardTelemetry.addData("target position", target_position);
        dashboardTelemetry.addData("boolean at target position", pid.atSetPoint());
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        current_position = this.motor0.getCurrentPosition();
        pid.setPID(kP,kI,kD);
        double output = 0;
        if (Math.abs(target_position - current_position) > 10){
            output = this.pid.calculate(current_position, target_position);
            output = limiter(output, maxPower);
            this.motor0.setPower(output);
        }




        dashboardTelemetry.addData("power", output);
        dashboardTelemetry.addData("current position", motor0.getCurrentPosition());
        dashboardTelemetry.addData("target position", target_position);
        dashboardTelemetry.addData("boolean at target position", pid.atSetPoint());


        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        this.motor0.setPower(0);
//        dashboardTelemetry.addData("power", motor0.getPower());
//        dashboardTelemetry.addData("current position", motor0.getCurrentPosition());
//        dashboardTelemetry.addData("target position", target_position);
//        dashboardTelemetry.addData("motor0 power", motor0.getPower());
        dashboardTelemetry.update();
    }

     /**
     * this will limit the input to a range of -limiter to limiter
     * @param input the value to be limited
     * @param limiter the max value the input can be
     * @return the limited input
     */
    public static double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }
}
