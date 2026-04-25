package org.firstinspires.ftc.teamcode.current;

public class ShooterMath {

    public static ShooterState calculateStationaryShot(double distanceToPoint) {
        double x = distanceToPoint;
        double y = ShooterConstants.SCORE_HEIGHT;
        double theta = ShooterConstants.SCORE_ANGLE;
        double g = ShooterConstants.GRAVITY;

        // 1. Calculate Launch Angle (Alpha)
        double term1 = (2 * y) / x;
        double term2 = Math.tan(theta);
        double launchAngle = Math.atan(term1 - term2);

        // 2. Calculate Launch Velocity (v0)
        double numerator = g * Math.pow(x, 2);
        double denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * ((x * Math.tan(launchAngle)) - y);
        double launchVelocity = Math.sqrt(numerator / denominator);

        return new ShooterState(launchAngle, launchVelocity);
    }
}
