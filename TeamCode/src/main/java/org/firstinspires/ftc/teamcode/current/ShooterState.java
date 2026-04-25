package org.firstinspires.ftc.teamcode.current;


public class ShooterState {
    public double angle;    // The target hood angle (in radians or degrees)
    public double velocity; // The target flywheel speed (in inches/sec)

    // Constructor: This lets you create the object easily
    public ShooterState(double angle, double velocity) {
        this.angle = angle;
        this.velocity = velocity;
    }
}
