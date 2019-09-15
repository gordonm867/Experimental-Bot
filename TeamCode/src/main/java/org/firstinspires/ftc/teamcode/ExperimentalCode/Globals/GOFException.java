package org.firstinspires.ftc.teamcode.ExperimentalCode.Globals;

public class GOFException extends RuntimeException {

    private String text;

    public GOFException(String err) {
        text = err;
    }

    public String toString() {
        return "516 Gears of Fire Exception: " + text;
    }
}
