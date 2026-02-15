package frc.robot.util;

import java.util.Optional;

/**
 * Linear interpolation of flywheel RPM based on distance to target tag.
 * Distance is from PhotonVision range estimation (tag 10).  
 * Tune kMinDistance, kMaxDistance, kMinRPM, kMaxRPM for your robot.
 */
public final class FlywheelInterpolation {

    /** Distance (meters) at which to use minimum RPM. */
    public static final double kMinDistance = 1.0;
    /** Distance (meters) at which to suse maximum RPM. */
    public static final double kMaxDistance = 50.0;
    /** RPM at minimum distance. */
    public static final double kMinRPM = 3000;
    /** RPM at maximum distance. */
    public static final double kMaxRPM = 3500;
    /** Default RPM when distance cannot be computed (tag not visible). */
    public static final double kDefaultRPM = 3000;

    private FlywheelInterpolation() {}

    /**
     * Linearly interpolates flywheel RPM from distance to target tag.
     *
     * @param distanceMeters Distance to tag in meters (from PhotonVision)
     * @return RPM to use for shooter
     */
    public static double interpolateRPM(double distanceMeters) {
        double clampedDistance = Math.max(kMinDistance, Math.min(kMaxDistance, distanceMeters));
        double t = (clampedDistance - kMinDistance) / (kMaxDistance - kMinDistance);
        return kMinRPM + t * (kMaxRPM - kMinRPM);
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
