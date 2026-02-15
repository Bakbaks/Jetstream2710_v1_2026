package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Optional;

public final class FlywheelInterpolation {

    /** Default RPM when distance cannot be computed (tag not visible). */
    public static final double kDefaultRPM = 3000;

    /** Shooter RPM lookup table - maps distance (meters) to RPM */
    private static final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

    // Initialize the table with distance-RPM sweet spots
    static {
        // distance-RPM pairs here (distance in meters, RPM)
        shooterTable.put(1.0, 3000.0);   // Close range
        shooterTable.put(2.0, 3100.0);
        shooterTable.put(3.0, 3200.0);
        shooterTable.put(4.0, 3300.0);
        shooterTable.put(5.0, 3400.0);   // Far range
    }

    private FlywheelInterpolation() {}

    /**
     * Gets interpolated RPM for the given distance.
     * Automatically performs linear interpolation between table entries.
     *
     * @param distanceMeters Distance to tag in meters (from PhotonVision)
     * @return RPM to use for shooter
     */
    public static double interpolateRPM(double distanceMeters) {
        return shooterTable.get(distanceMeters);
    }

    /**
     * Gets interpolated RPM from PhotonVision distance. Uses default RPM if tag not visible.
     *
     * @param distanceMeters Optional distance from Vision.getDistanceToTag10()
     * @return RPM to use for shooter
     */
    public static double getRPMForDistance(Optional<Double> distanceMeters) {
        return distanceMeters
                .map(FlywheelInterpolation::interpolateRPM)
                .orElse(kDefaultRPM);
    }
}
