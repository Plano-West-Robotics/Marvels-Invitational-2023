package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class TeleOp extends OpMode {
    Claw claw;
    Lift lift;

    @Override
    public void init() {
        claw = new Claw(telemetry, hardwareMap);
        lift = new Lift(telemetry, hardwareMap);
        lift.setManual(true);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_down) {
            lift.setManual(false);
        } else if (gamepad2.dpad_up) {
            lift.setManual(true);
        }

        if (gamepad2.a) {
            lift.goTo(Lift.POS_DOWN);
        } else if (gamepad2.b) {
            lift.goTo(Lift.POS_LOW);
        } else if (gamepad2.x) {
            lift.goTo(Lift.POS_MID);
        } else if (gamepad2.y) {
            lift.goTo(Lift.POS_HIGH);
        }

        if (gamepad2.left_bumper) {
            claw.goTo(Claw.POS_CLOSE);
        } else if (gamepad2.right_bumper) {
            claw.goTo(Claw.POS_OPEN);
        }

        lift.update(gamepad2.left_stick_y);
    }
}
