package org.firstinspires.ftc.teamcode.current;

public class ShooterSolution {
    public double hoodAngle;       // Angle of the hood (degrees)
    public double flywheelVelocity; // Speed of flywheel (inches/sec or RPM)
    public double turretAngle;     // Field-centric angle the turret should face (degrees)

    public ShooterSolution(double hoodAngle, double flywheelVelocity, double turretAngle) {
        this.hoodAngle = hoodAngle;
        this.flywheelVelocity = flywheelVelocity;
        this.turretAngle = turretAngle;
    }
}
