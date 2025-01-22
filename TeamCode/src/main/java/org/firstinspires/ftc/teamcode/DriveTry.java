package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp
@Config
public class DriveTry extends OpMode {
    DriveTrain train = new DriveTrain();
    Lift lift = new Lift();
    Arm arm = new Arm();
    public double g1LeftStickY = 0.0;
    public double g1LeftStickX = 0.0;

    public double g1RightStickY = 0.0;
    public double g1RightStickX = 0.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();





    @Override
    public void init(){
        train.init(hardwareMap);
        lift.init(hardwareMap);
        arm.init(hardwareMap);
    }
    @Override
    public void loop(){
        gamePadInput();
        train.getController(g1LeftStickX,g1RightStickX,g1LeftStickY, g1RightStickY);
        train.Drive();
        lift.Ascend(gamepad2.right_trigger, gamepad2.left_trigger);
        arm.mooveShoulder(gamepad2.right_bumper, gamepad2.left_bumper);
        arm.mooveArm(gamepad2.a, gamepad2.b);

        dashboardTelemetry.addData("tight trigger", gamepad1.right_trigger);
        dashboardTelemetry.addData("left trigger", gamepad1.left_trigger);
        dashboardTelemetry.update();

    }


    private void gamePadInput(){
        g1LeftStickY =gamepad1.left_stick_y *-1;
        g1LeftStickX =gamepad1.left_stick_x;

        g1RightStickY =gamepad1.right_stick_y *-1;
        g1RightStickX =gamepad1.right_stick_x;
    }
}
