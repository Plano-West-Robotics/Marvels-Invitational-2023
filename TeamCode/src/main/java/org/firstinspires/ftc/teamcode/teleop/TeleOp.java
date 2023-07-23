package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.Arm;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {

    //region Private Members
    Claw claw;
    Lift lift;
    Drive drive;
    Arm arm;

    GamepadEx gp1;
    GamepadEx gp2;
    //endregion

    static class Buttons {
        //region SecondaryController
        static final int liftManual = GamepadEx.RIGHT_BUMPER;
        static final int liftPower = GamepadEx.LEFT_STICK_Y;
        /*static final int liftDown = GamepadEx.A;
        static final int liftLow = GamepadEx.B;
        static final int liftMid = GamepadEx.X;
        static final int liftHigh = GamepadEx.Y;*/
        static final int liftDown = GamepadEx.DPAD_DOWN;
        static final int liftLow = GamepadEx.DPAD_RIGHT;
        static final int liftMid = GamepadEx.DPAD_LEFT;
        static final int liftHigh = GamepadEx.DPAD_UP;

        static final int claw = GamepadEx.B; // kinda arbitrary, may change later
        //static final int claw = GamepadEx.DPAD_LEFT;
        //static final int armPickup = GamepadEx.DPAD_DOWN;
        //static final int armDrop = GamepadEx.DPAD_UP;
        static final int armPickup = GamepadEx.A;
        static final int armDrop= GamepadEx.Y;
        //endregion
        //region MainController
        static final int driveX = GamepadEx.LEFT_STICK_X;
        static final int driveY = GamepadEx.LEFT_STICK_Y;
        static final int driveTheta = GamepadEx.RIGHT_STICK_X;
        //endregion
    }

    @Override
    public void init() {
        claw = new Claw(telemetry, hardwareMap);
        lift = new Lift(telemetry, hardwareMap);
        drive = new Drive(hardwareMap, telemetry);
        arm = new Arm(telemetry, hardwareMap);

        lift.setManual(true);

        gp1 = new GamepadEx();
        gp2 = new GamepadEx();

        gp1.update(gamepad1);
        gp2.update(gamepad2);
    }

    @Override
    public void loop() {

        //region Lift
        if (gp2.isPressed(Buttons.liftDown)) {
            lift.goTo(Lift.POS_DOWN);
        } else if (gp2.isPressed(Buttons.liftLow)) {
            lift.goTo(Lift.POS_LOW);
        } else if (gp2.isPressed(Buttons.liftMid)) {
            lift.goTo(Lift.POS_MID);
        } else if (gp2.isPressed(Buttons.liftHigh)) {
            lift.goTo(Lift.POS_HIGH);
        }
        if (gp2.justPressed(Buttons.liftManual)) {
            lift.toggleManual();
        }
        //endregion

        //region Arm
        if (gp2.justPressed(Buttons.claw)) {
            claw.toggle();
        }

        if (gp2.isPressed(Buttons.armPickup)) {
            claw.goTo(Claw.POS_CLOSE);
            arm.goTo(Arm.POS_PICKUP);
        } else if (gp2.isPressed(Buttons.armDrop)) {
            arm.goTo(Arm.POS_DROP);
        }
        //endregion

        lift.update(gp2.getValue(Buttons.liftPower));
        drive.update(gp1.getValue(Buttons.driveX), gp1.getValue(Buttons.driveY), -gp1.getValue(Buttons.driveTheta));
        gp1.update(gamepad1);
        gp2.update(gamepad2);

        telemetry.update();
    }
}
