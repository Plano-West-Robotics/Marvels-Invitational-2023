package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    private Gamepad prevGamepad;
    private Gamepad curGamepad;

    public static final int A = 0;
    public static final int B = 1;
    public static final int X = 2;
    public static final int Y = 3;
    public static final int RIGHT_BUMPER = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_TRIGGER = 6;
    public static final int LEFT_TRIGGER = 7;
    public static final int RIGHT_STICK_BUTTON = 8;
    public static final int RIGHT_STICK_X = 9;
    public static final int RIGHT_STICK_Y = 10;
    public static final int LEFT_STICK_BUTTON = 11;
    public static final int LEFT_STICK_X = 12;
    public static final int LEFT_STICK_Y = 13;
    public static final int DPAD_UP = 14;
    public static final int DPAD_DOWN = 15;
    public static final int DPAD_LEFT = 16;
    public static final int DPAD_RIGHT = 17;

    public GamepadEx() {
        prevGamepad = new Gamepad();
        curGamepad = new Gamepad();
    }

    public boolean isPressed(int button) {
        return isPressed(button, curGamepad);
    }

    private boolean isPressed(int button, Gamepad gamepad) {
        switch (button) {
            case GamepadEx.A: return gamepad.a;
            case GamepadEx.B: return gamepad.b;
            case GamepadEx.X: return gamepad.x;
            case GamepadEx.Y: return gamepad.y;
            case GamepadEx.RIGHT_BUMPER: return gamepad.right_bumper;
            case GamepadEx.LEFT_BUMPER: return gamepad.left_bumper;
            case GamepadEx.RIGHT_TRIGGER: return gamepad.right_trigger > 0;
            case GamepadEx.LEFT_TRIGGER: return gamepad.left_trigger > 0;
            case GamepadEx.RIGHT_STICK_BUTTON: return gamepad.right_stick_button;
            case GamepadEx.RIGHT_STICK_X: return gamepad.right_stick_x > 0;
            case GamepadEx.RIGHT_STICK_Y: return gamepad.right_stick_y > 0;
            case GamepadEx.LEFT_STICK_BUTTON: return gamepad.left_stick_button;
            case GamepadEx.LEFT_STICK_X: return gamepad.left_stick_x > 0;
            case GamepadEx.LEFT_STICK_Y: return gamepad.left_stick_y > 0;
            case GamepadEx.DPAD_UP: return gamepad.dpad_up;
            case GamepadEx.DPAD_DOWN: return gamepad.dpad_down;
            case GamepadEx.DPAD_LEFT: return gamepad.dpad_left;
            case GamepadEx.DPAD_RIGHT: return gamepad.dpad_right;
            default:
                throw new RuntimeException("Gamepad argument doesn't exist");
        }
    }

    public float getValue(int button) {
        return getValue(button, curGamepad);
    }

    private float getValue(int button, Gamepad gamepad) {
        switch (button) {
            case GamepadEx.LEFT_TRIGGER: return gamepad.left_trigger;
            case GamepadEx.RIGHT_TRIGGER: return gamepad.right_trigger;
            case GamepadEx.LEFT_STICK_X: return gamepad.left_stick_x;
            case GamepadEx.LEFT_STICK_Y: return gamepad.left_stick_y;
            case GamepadEx.RIGHT_STICK_X: return gamepad.right_stick_x;
            case GamepadEx.RIGHT_STICK_Y: return gamepad.right_stick_y;
            default:
                throw new RuntimeException("Cannot fetch analog value from that button");
        }
    }

    public boolean justPressed(int button) {
        return (!isPressed(button, prevGamepad) && isPressed(button, curGamepad));
    }

    public void update(Gamepad gamepad) {
        try {
            prevGamepad.copy(curGamepad);
            curGamepad.copy(gamepad);
        } catch (Exception ignored) {

        }
    }
}
