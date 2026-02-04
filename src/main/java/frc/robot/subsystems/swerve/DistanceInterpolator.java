package frc.robot.subsystems.swerve;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class DistanceInterpolator {
    private final NavigableMap<Double, Double> distanceToRPM = new TreeMap<>();

    public DistanceInterpolator() {
        populateDefaultMap();
    }

    private void populateDefaultMap() {
        // Example tuning points (meters -> RPM)
        distanceToRPM.put(1.5, 1800.0);
        distanceToRPM.put(2.0, 2100.0);
        distanceToRPM.put(3.0, 2600.0);
        distanceToRPM.put(4.5, 3200.0);
    }

    public double getRPMForDistance(double distanceMeters) {
        if (distanceToRPM.isEmpty()) {
            return 0.0;
        }

        Map.Entry<Double, Double> lower = distanceToRPM.floorEntry(distanceMeters);
        Map.Entry<Double, Double> upper = distanceToRPM.ceilingEntry(distanceMeters);

        if (lower == null && upper == null) {
            return 0.0;
        } else if (lower == null) {
            return upper.getValue();
        } else if (upper == null) {
            return lower.getValue();
        } else if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        } else {
            double x0 = lower.getKey();
            double y0 = lower.getValue();
            double x1 = upper.getKey();
            double y1 = upper.getValue();
            double t = (distanceMeters - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }
    }

    public void putTuningPoint(double distanceMeters, double rpm) {
        distanceToRPM.put(distanceMeters, rpm);
    }

    public void removeTuningPoint(double distanceMeters) {
        distanceToRPM.remove(distanceMeters);
    }
}
