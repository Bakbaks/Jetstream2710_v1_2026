package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ShotConstants;

public final class RobotLocalization {

    private static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double r_width = 38;
    public static final double r_length = 30;

    private RobotLocalization() {}

    public static Transform2d robotToTargetTransform(Pose2d robotPose, Pose2d targetPose) {
        return targetPose.minus(robotPose);
    }

    public static Translation2d robotToTargetTranslation(Pose2d robotPose, Pose2d targetPose) {
        return robotToTargetTransform(robotPose, targetPose).getTranslation();
    }

    public static double robotToTargetDistanceMeters(Pose2d robotPose, Pose2d targetPose) {
        return robotToTargetTranslation(robotPose, targetPose).getNorm();
    }

    public static Rotation2d robotToTargetBearing(Pose2d robotPose, Pose2d targetPose) {
        Translation2d t = robotToTargetTranslation(robotPose, targetPose);
        return t.getAngle();
    }

    public static Pose2d robotToShooterExitPose(Pose2d robotPose) {
        return robotPose.plus(ShotConstants.RobotToShooterExit);
    }

    public static Translation2d shooterExitToTargetTranslation(Pose2d robotPose, Pose2d targetPose) {
        Pose2d shooterExitPose = robotToShooterExitPose(robotPose);
        return targetPose.getTranslation().minus(shooterExitPose.getTranslation());
    }

    public static double shooterExitToTargetDistanceMeters(Pose2d robotPose, Pose2d targetPose) {
        return shooterExitToTargetTranslation(robotPose, targetPose).getNorm();
    }

    public static Rotation2d shooterExitToTargetBearing(Pose2d robotPose, Pose2d targetPose) {
        return shooterExitToTargetTranslation(robotPose, targetPose).getAngle();
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