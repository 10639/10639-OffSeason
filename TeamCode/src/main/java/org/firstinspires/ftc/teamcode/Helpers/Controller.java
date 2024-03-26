package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.EnumMap;

public class Controller {

    public enum Button {
        Y, X, A, B, LEFT_BUMPER, RIGHT_BUMPER, BACK, START,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        TRIANGLE, SQUARE, CROSS, CIRCLE, SHARE, OPTIONS
    }

    public enum Trigger {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    public class ButtonReader {
        private boolean oldState = false, newState = false;
        private Gamepad gamepad;
        private Button button;

        public ButtonReader(Button button, Gamepad gamepad) {
            this.button = button;
            this.gamepad = gamepad;
        }

        public void read() {
            oldState = newState;
            newState = getButton(button, gamepad);
        }

        public boolean isPressed() {
            return newState;
        }

        public boolean isReleased() {
            return (!newState);
        }

        public boolean justPressed() {
            return ((!oldState) && (newState));
        }

        public boolean justReleased() {
            return ((oldState) && (!newState));
        }

        public boolean justChanged() {
            return (oldState != newState);
        }
    }

    private final EnumMap<Button, ButtonReader> buttonReaders = new EnumMap<>(Button.class);
    private Gamepad gamepad;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
        for (Button button : Button.values()) {
            buttonReaders.put(button, new ButtonReader(button, gamepad));
        }
    }

    private boolean getButton(Button button, Gamepad gamepad) {
        boolean buttonValue = false;
        switch (button) {
            case Y:
            case TRIANGLE:
                buttonValue = gamepad.y || gamepad.triangle;
                break;
            case X:
            case SQUARE:
                buttonValue = gamepad.x || gamepad.square;
                break;
            case A:
            case CROSS:
                buttonValue = gamepad.a || gamepad.cross;
                break;
            case B:
            case CIRCLE:
                buttonValue = gamepad.b || gamepad.circle;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonValue = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonValue = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonValue = gamepad.dpad_right;
                break;
            case BACK:
            case SHARE:
                buttonValue = gamepad.back || gamepad.share;
                break;
            case START:
            case OPTIONS:
                buttonValue = gamepad.start || gamepad.options;
                break;
            case LEFT_STICK_BUTTON:
                buttonValue = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonValue = gamepad.right_stick_button;
                break;
            default:
                break;
        }
        return buttonValue;
    }

    public boolean getButton(Button button) {
        return getButton(button, gamepad);
    }

    private double getTrigger(Trigger trigger) {
        return trigger == Trigger.LEFT_TRIGGER ? gamepad.left_trigger : gamepad.right_trigger;
    }

    public double getLeftY() {
        return -gamepad.left_stick_y;
    }

    public double getRightY() {
        return -gamepad.right_stick_y;
    }

    public double getLeftX() {
        return gamepad.left_stick_x;
    }

    public double getRightX() {
        return gamepad.right_stick_x;
    }

    public void readButtons() {
        for (Button button : Button.values()) {
            buttonReaders.get(button).read();
        }
    }

    public boolean isPressed(Button button) { //Use this to check if a button is currently being held down.
        return buttonReaders.get(button).isPressed();
    }

    public boolean isReleased(Button button) { //Use this to check if a button is not currently being held down.
        return buttonReaders.get(button).isReleased();
    }

    public boolean justPressed(Button button) { //Use this to check if a button was just pressed down (not necessarily being held down).
        return buttonReaders.get(button).justPressed();
    }

    public boolean justReleased(Button button) { //Use this to check if a button was just released (was being held down but is not anymore).
        return buttonReaders.get(button).justReleased();
    }

    public boolean justChanged(Button button) { //Use this to check if the state of a button has just changed (either pressed or released).
        return buttonReaders.get(button).justChanged();
    }

}
