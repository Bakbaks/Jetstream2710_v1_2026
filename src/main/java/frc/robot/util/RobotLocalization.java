package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Linear interpolation of flywheel RPM based on distance to target tag.
 * Distance is from PhotonVision range estimation (tag 10).  
 * Tune kMinDistance, kMaxDistance, kMinRPM, kMaxRPM for your robot.
 */
public final class RobotLocalization {


    private static final AprilTagFieldLayout kFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);


    /** Distance (meters) at which to use minimum RPM. */
    public static final double r_width = 27;
    public static final double r_length = 27;


    private RobotLocalization() {}

    public static Transform2d robotToTargetTransform(Pose2d robotPose, Pose2d targetPose) {
        return targetPose.minus(robotPose);  // robot -> target (robot frame)
    }

    public static Translation2d robotToTargetTranslation(Pose2d robotPose, Pose2d targetPose) {
        return robotToTargetTransform(robotPose, targetPose).getTranslation();
    }

    public static double robotToTargetDistanceMeters(Pose2d robotPose, Pose2d targetPose) {
        return robotToTargetTranslation(robotPose, targetPose).getNorm();
    }

    // 0 deg = straight ahead of robot
    // +90 deg = left of robot (WPILib's +Y is left)
    // -90 deg = right of robot
    public static Rotation2d robotToTargetBearing(Pose2d robotPose, Pose2d targetPose) {
        Translation2d t = robotToTargetTranslation(robotPose, targetPose);
        return t.getAngle(); // atan2(y, x)
    }

    public static Optional<Pose2d> fieldPoseFromTagTransform(int tagId, Transform2d tagToPoint) {
        Optional<Pose3d> tagPose3d = kFieldLayout.getTagPose(tagId);
        if (tagPose3d.isEmpty()) {
        return Optional.empty();
        }

        Pose2d tagPose2d = tagPose3d.get().toPose2d();
        Pose2d pointPoseField = tagPose2d.plus(tagToPoint);
        return Optional.of(pointPoseField);
    }

}
