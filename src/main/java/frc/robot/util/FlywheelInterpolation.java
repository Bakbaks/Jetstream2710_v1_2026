package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Vision;

/**
 * Linear interpolation of flywheel RPM based on distance to target tag.
 * Tune kMinDistance, kMaxDistance, kMinRPM, kMaxRPM for your robot.
 */
public final class FlywheelInterpolation {

    /** Distance (meters) at which to use minimum RPM. */
    public static final double kMinDistance = 1.0;
    /** Distance (meters) at which to use maximum RPM. */
    public static final double kMaxDistance = 4.0;
    /** RPM at minimum distance. */
    public static final double kMinRPM = 800;
    /** RPM at maximum distance. */
    public static final double kMaxRPM = 1200;
    /** Default RPM when distance cannot be computed. */
    public static final double kDefaultRPM = 1000;

    /** Tag ID to target for shooter (matches BasicRotate alignment tag). */
    public static final int kTargetTagId = 10;

    private FlywheelInterpolation() {}

    /**
     * Computes distance from robot pose to the target tag in meters.
     *
     * @param robotPose Current robot pose
     * @return Optional containing distance in meters, or empty if tag not in layout
     */
    public static Optional<Double> getDistanceToTag(Pose2d robotPose) {
        var tagPose = Vision.kTagLayout.getTagPose(kTargetTagId);
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }
        Translation2d tagTranslation = tagPose.get().toPose2d().getTranslation();
        double distance = robotPose.getTranslation().getDistance(tagTranslation);
        return Optional.of(distance);
    }

    /**
     * Linearly interpolates flywheel RPM from distance to target tag.
     *
     * @param distanceMeters Distance to tag in meters
     * @return RPM to use for shooter
     */
    public static double interpolateRPM(double distanceMeters) {
        double clampedDistance = Math.max(kMinDistance, Math.min(kMaxDistance, distanceMeters));
        double t = (clampedDistance - kMinDistance) / (kMaxDistance - kMinDistance);
        return kMinRPM + t * (kMaxRPM - kMinRPM);
    }

    /**
     * Gets interpolated RPM based on robot pose. Uses default RPM if distance cannot be computed.
     *
     * @param robotPose Current robot pose
     * @return RPM to use for shooter
     */
    public static double getRPMForPose(Pose2d robotPose) {
        return getDistanceToTag(robotPose)
                .map(FlywheelInterpolation::interpolateRPM)
                .orElse(kDefaultRPM);
    }
}
