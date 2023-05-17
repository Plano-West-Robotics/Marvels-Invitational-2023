package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class TeleOp extends OpMode {
    Claw claw;
    Lift lift;

    GamepadEx gp1;
    GamepadEx gp2;

    static class Buttons {
        static final int liftManual = GamepadEx.DPAD_UP;
        static final int liftPower = GamepadEx.LEFT_STICK_Y;
        static final int liftDown = GamepadEx.A;
        static final int liftLow = GamepadEx.B;
        static final int liftMid = GamepadEx.X;
        static final int liftHigh = GamepadEx.Y;

        static final int claw = GamepadEx.LEFT_BUMPER;
    }

    @Override
    public void init() {
        claw = new Claw(telemetry, hardwareMap);
        lift = new Lift(telemetry, hardwareMap);
        lift.setManual(true);

        gp1 = new GamepadEx();
        gp2 = new GamepadEx();

        gp1.update(gamepad1);
        gp2.update(gamepad2);
    }

    @Override
    public void loop() {
        if (gp2.isPressed(Buttons.liftManual)) {
            lift.toggleManual();
        }

        if (gp2.isPressed(Buttons.liftDown)) {
            lift.goTo(Lift.POS_DOWN);
        } else if (gp2.isPressed(Buttons.liftLow)) {
            lift.goTo(Lift.POS_LOW);
        } else if (gp2.isPressed(Buttons.liftMid)) {
            lift.goTo(Lift.POS_MID);
        } else if (gp2.isPressed(Buttons.liftHigh)) {
            lift.goTo(Lift.POS_HIGH);
        }

        if (gp2.justPressed(Buttons.claw)) {
            claw.toggle();
        }

        lift.update(gp2.getValue(Buttons.liftPower));

        gp1.update(gamepad1);
        gp2.update(gamepad2);
    }
}
