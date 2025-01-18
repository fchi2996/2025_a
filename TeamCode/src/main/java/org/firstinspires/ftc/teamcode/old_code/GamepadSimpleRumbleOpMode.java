package org.firstinspires.ftc.teamcode.old_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp()
public class GamepadSimpleRumbleOpMode extends OpMode {
    @Override
    public void init() {
    }

@Override
public void loop() {
        if (gamepad1.a) {
            gamepad1.rumble(50);
        }
        if (gamepad1.b) {
        gamepad1.rumble(100);
        }
    }
}