package org.firstinspires.ftc.teamcode.ExperimentalCode.Globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Globals {
    public static final double DRIVE_RADIUS = 4 * Math.PI; // Drive wheel radius
    public static final double OMNI_RADIUS = 3 * Math.PI; // Omni wheel radius

    public static final double DRIVE_FEET_PER_TICK = (DRIVE_RADIUS / (560 * 12)); // Feet moved per drive wheel encoder tick
    public static final double OMNI_FEET_PER_TICK = (OMNI_RADIUS / (1440 * 12)); // Feet moved per omni wheel encoder tick

    public static double MAX_SPEED = 1.0; // Maximum proportion of speed allowed

    public static double START_X = -4; // Default starting x position
    public static double START_Y = -2; // Default starting y position
    public static double START_THETA = 180; // Default starting angle

    public static double boxDown = 0.395;
    public static double boxNeutral = 0.45;
    public static double clampOpen = 0.355;
    public static double clampClose = 0;
    public static double clampSide = 0;
    public static double clampNeutral = 0.25;
    public static double clampGrab = 0; // possibly wrong name
    public static double clampPlace = 0.93; // possibly wrong name
    public static int EXTEND_POS = -2250;

    public static double xOffset = 2;
    public static double yOffset = 0.5;

    public static double minB = 6;
    public static double minG = 170;
    public static double minR = 30;

    public static double maxB = 40;
    public static double maxG = 255;
    public static double maxR = 230;

    public static double BLUR_CONSTANT = 25;

}