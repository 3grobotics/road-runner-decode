package org.firstinspires.ftc.teamcode.current;

public class ShooterConstants {
    // Field Constants (Measure these on your field)
    public static final double GOAL_X = 72.0; // inches
    public static final double GOAL_Y = 36.0; // inches

    // Tuning Parameters (Adjust these during testing)
    public static double SCORE_HEIGHT = 16.0; // Height of point above the launcher
    public static double SCORE_ANGLE = Math.toRadians(-45); // Downward angle at entry
    public static double POINT_RADIUS = 4.0; // Distance from goal center to the point

    // Physics Constants
    public static final double GRAVITY = 386.1; // in/s^2 (approx 9.8 m/s^2)

    // Turret Hardware Constraints
    public static final double TURRET_RANGE_DEGREES = 303.5;
    public static final double TURRET_HOME_POSITION = 0.5; // Center of servo travel (facing forward)

    // Safety Limits (Don't hit hard stops)
    public static final double TURRET_MIN_POS = 0.02;
    public static final double TURRET_MAX_POS = 0.98;
}


