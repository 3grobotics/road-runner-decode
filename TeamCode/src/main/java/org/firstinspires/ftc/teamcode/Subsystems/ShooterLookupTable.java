package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookupTable {

    // Store the data: Distance -> {Hood Position, Flywheel Velocity}
    // We use a Double[] array where index 0 = Hood, index 1 = Velocity
    private final TreeMap<Double, double[]> table = new TreeMap<>();

    // Add a known "perfect shot" point
    public void add(double distance, double hoodPos, double velocity) {
        table.put(distance, new double[]{hoodPos, velocity});
    }

    // The magic function: Get calculated settings for ANY distance
    public double[] get(double distance) {
        // 1. Exact match?
        if (table.containsKey(distance)) return table.get(distance);

        // 2. Find the closest known distances below and above our current distance
        Map.Entry<Double, double[]> floor = table.floorEntry(distance);
        Map.Entry<Double, double[]> ceiling = table.ceilingEntry(distance);

        // Handle edge cases (if we are closer than min or further than max)
        if (floor == null) return ceiling.getValue(); // Cap at minimum
        if (ceiling == null) return floor.getValue(); // Cap at maximum

        // 3. Linear Interpolation (LERP)
        double lowerDist = floor.getKey();
        double upperDist = ceiling.getKey();
        double[] lowerVal = floor.getValue();
        double[] upperVal = ceiling.getValue();

        // Calculate percentage (t) between the two points (0.0 to 1.0)
        double t = (distance - lowerDist) / (upperDist - lowerDist);

        // Interpolate Hood (Index 0)
        double interpHood = lowerVal[0] + (t * (upperVal[0] - lowerVal[0]));

        // Interpolate Velocity (Index 1)
        double interpVel = lowerVal[1] + (t * (upperVal[1] - lowerVal[1]));

        return new double[]{interpHood, interpVel};
    }
}