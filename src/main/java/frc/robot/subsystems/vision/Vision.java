/*
* MIT License
*
* Copyright (c) PhotonVision
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonPoseEstimator photonEstimator1;
    private final PhotonPoseEstimator photonEstimator2;
    private Matrix<N3, N1> curStdDevs1;
    private Matrix<N3, N1> curStdDevs2;
    private final EstimateConsumer estConsumer;
    private final DoubleSupplier robotPitchRadians;

    /** Latest camera1 result from {@link #periodic()} (same data path as {@code getAllUnreadResults}). */
    private PhotonPipelineResult latestCamera1Result;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private record PoseFrame(EstimatedRobotPose pose, boolean multiTag) {}

    public Vision(EstimateConsumer estConsumer, DoubleSupplier robotPitchRadians) {
        this.estConsumer = estConsumer;
        this.robotPitchRadians = robotPitchRadians;
        camera1 = new PhotonCamera(kCameraName1);
        camera2 = new PhotonCamera(kCameraName2);

        photonEstimator1 = new PhotonPoseEstimator(kTagLayout, kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(kTagLayout, kRobotToCam2);

        curStdDevs1 = kVisionSingleTagTrustStdDevs;
        curStdDevs2 = kVisionSingleTagTrustStdDevs;
    }

    @Override
    public void periodic() {
        final double tiltGateRad = Units.degreesToRadians(kVisionTiltGatePitchDeg);
        final boolean tiltOk = Math.abs(robotPitchRadians.getAsDouble()) <= tiltGateRad;

        var cam1Unread = camera1.getAllUnreadResults();
        for (var change : cam1Unread) {
            processCameraFrame(photonEstimator1, change, tiltOk, 1.0, true);
        }
        if (!cam1Unread.isEmpty()) {
            latestCamera1Result = cam1Unread.get(cam1Unread.size() - 1);
        }

        for (var change : camera2.getAllUnreadResults()) {
            processCameraFrame(photonEstimator2, change, tiltOk, kCamera2VisionStdDevScale, false);
        }
    }

    private void processCameraFrame(
            PhotonPoseEstimator estimator,
            PhotonPipelineResult result,
            boolean tiltOk,
            double cameraStdDevScale,
            boolean camera1) {
        if (!tiltOk) {
            return;
        }
        Optional<PoseFrame> frameOpt = estimatePoseFrame(estimator, result);
        if (frameOpt.isEmpty()) {
            return;
        }
        PoseFrame frame = frameOpt.get();
        List<PhotonTrackedTarget> targets = result.getTargets();
        if (!passesAmbiguityGate(targets, frame.multiTag)) {
            return;
        }

        Matrix<N3, N1> stdDevs =
                computeVisionTrustStdDevs(estimator, frame.pose, frame.multiTag, targets, cameraStdDevScale);
        if (camera1) {
            curStdDevs1 = stdDevs;
        } else {
            curStdDevs2 = stdDevs;
        }
        estConsumer.accept(
                frame.pose.estimatedPose.toPose2d(),
                frame.pose.timestampSeconds,
                stdDevs);
    }

    private static Optional<PoseFrame> estimatePoseFrame(
            PhotonPoseEstimator estimator, PhotonPipelineResult result) {
        var multiTag = estimator.estimateCoprocMultiTagPose(result);
        if (multiTag.isPresent()) {
            return Optional.of(new PoseFrame(multiTag.get(), true));
        }
        return estimator.estimateLowestAmbiguityPose(result).map(p -> new PoseFrame(p, false));
    }

    private static boolean passesAmbiguityGate(List<PhotonTrackedTarget> targets, boolean multiTag) {
        if (targets.isEmpty()) {
            return false;
        }
        double[] finite =
                targets.stream()
                        .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                        .filter(Double::isFinite)
                        .toArray();
        if (finite.length == 0) {
            return false;
        }
        if (multiTag) {
            return Arrays.stream(finite).allMatch(a -> a <= kMaxTagPoseAmbiguity);
        }
        return Arrays.stream(finite).min().orElse(Double.POSITIVE_INFINITY) <= kMaxTagPoseAmbiguity;
    }

    private static Matrix<N3, N1> computeVisionTrustStdDevs(
            PhotonPoseEstimator estimator,
            EstimatedRobotPose estimatedPose,
            boolean multiTag,
            List<PhotonTrackedTarget> targets,
            double cameraStdDevScale) {
        Matrix<N3, N1> base = multiTag ? kVisionMultiTagTrustStdDevs : kVisionSingleTagTrustStdDevs;
        double avgDist = averageTagDistanceMeters(estimator, estimatedPose, targets);
        double distScale = 1.0;
        if (avgDist > kVisionDistancePenaltyThresholdM) {
            distScale =
                    Math.exp(
                            kVisionDistancePenaltyExpPerMeter
                                    * (avgDist - kVisionDistancePenaltyThresholdM));
        }
        return base.times(cameraStdDevScale * distScale);
    }

    private static double averageTagDistanceMeters(
            PhotonPoseEstimator estimator, EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        int numTags = 0;
        double sumDist = 0.0;
        var robot2d = estimatedPose.estimatedPose.toPose2d();
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            sumDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(robot2d.getTranslation());
        }
        return numTags == 0 ? 0.0 : sumDist / numTags;
    }

    public Optional<Double> getDistanceToTag(int tagId) {
        if (latestCamera1Result == null) {
            return Optional.empty();
        }
        for (var target : latestCamera1Result.getTargets()) {
            if (target.getFiducialId() == tagId) {
                return Optional.of(target.getBestCameraToTarget().getTranslation().getNorm());
            }
        }
        return Optional.empty();
    }

    public Optional<Double> getDistanceToTag10() {
        return getDistanceToTag(10);
    }

    /**
     * Last vision std devs passed to the pose filter for camera 1 (meters, radians); useful for
     * telemetry.
     */
    public Matrix<N3, N1> getEstimationStdDevs1() {
        return curStdDevs1;
    }

    public Matrix<N3, N1> getEstimationStdDevs2() {
        return curStdDevs2;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose) {
        visionSim.resetRobotPose(pose);
    }

    public Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
