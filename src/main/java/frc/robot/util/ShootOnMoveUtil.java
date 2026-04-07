package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShotConstants;

public final class ShootOnMoveUtil {
    private ShootOnMoveUtil() {}

    public static class ShotSolution {
        public final Pose2d latencyCompRobotPose;
        public final Pose2d shooterExitPose;
        public final Translation2d shooterExitVelocityField;
        public final Translation2d predictedExitPoint;
        public final double rawDistance;
        public final double effectiveDistance;
        public final double timeOfFlight;
        public final Rotation2d desiredHeadingField;

        public ShotSolution(
                Pose2d latencyCompRobotPose,
                Pose2d shooterExitPose,
                Translation2d shooterExitVelocityField,
                Translation2d predictedExitPoint,
                double rawDistance,
                double effectiveDistance,
                double timeOfFlight,
                Rotation2d desiredHeadingField) {
            this.latencyCompRobotPose = latencyCompRobotPose;
            this.shooterExitPose = shooterExitPose;
            this.shooterExitVelocityField = shooterExitVelocityField;
            this.predictedExitPoint = predictedExitPoint;
            this.rawDistance = rawDistance;
            this.effectiveDistance = effectiveDistance;
            this.timeOfFlight = timeOfFlight;
            this.desiredHeadingField = desiredHeadingField;
        }
    }

    public static ShotSolution solve(
            Pose2d robotPose,
            ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetPointField) {

        // 1) latency compensation
        Pose2d latencyCompRobotPose = new Pose2d(
                robotPose.getX() + fieldRelativeSpeeds.vxMetersPerSecond * ShotConstants.kShotLatencySeconds,
                robotPose.getY() + fieldRelativeSpeeds.vyMetersPerSecond * ShotConstants.kShotLatencySeconds,
                robotPose.getRotation().plus(
                        Rotation2d.fromRadians(
                                fieldRelativeSpeeds.omegaRadiansPerSecond * ShotConstants.kShotLatencySeconds)));

        // 2) shooter exit pose
        Pose2d shooterExitPose = latencyCompRobotPose.plus(ShotConstants.RobotToShooterExit);

        // 3) shooter exit velocity = robot translational + tangential from omega
        Translation2d rField = ShotConstants.RobotToShooterExit.getTranslation()
                .rotateBy(latencyCompRobotPose.getRotation());

        double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;

        double tangentialVx = -omega * rField.getY();
        double tangentialVy =  omega * rField.getX();

        Translation2d shooterExitVelocityField = new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentialVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentialVy);

        // 4) raw distance from shooter exit to target
        double rawDistance = shooterExitPose.getTranslation().getDistance(targetPointField);

        // 5) iterate TOF and predicted exit point
        double tof = FlywheelInterpolation.interpolateTOF(rawDistance);
        Translation2d predictedExitPoint = shooterExitPose.getTranslation();
        double effectiveDistance = rawDistance;

        for (int i = 0; i < ShotConstants.kShotPredictionIterations; i++) {
            predictedExitPoint = shooterExitPose.getTranslation().plus(
                    shooterExitVelocityField.times(tof));

            effectiveDistance = predictedExitPoint.getDistance(targetPointField);
            tof = FlywheelInterpolation.interpolateTOF(effectiveDistance);
        }

        // 6) desired robot heading so back of robot faces target
        Rotation2d desiredHeadingField = targetPointField
                .minus(predictedExitPoint)
                .getAngle()
                .plus(Rotation2d.fromDegrees(180.0));

        return new ShotSolution(
                latencyCompRobotPose,
                shooterExitPose,
                shooterExitVelocityField,
                predictedExitPoint,
                rawDistance,
                effectiveDistance,
                tof,
                desiredHeadingField);
    }
}