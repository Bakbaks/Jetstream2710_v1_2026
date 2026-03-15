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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator; 
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
 
public class Vision extends SubsystemBase {
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonPoseEstimator photonEstimator1;
    private final PhotonPoseEstimator photonEstimator2;
    private Matrix<N3, N1> curStdDevs1;
    private Matrix<N3, N1> curStdDevs2;
    private final EstimateConsumer estConsumer;

 
    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
 
    /**
    * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
    *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
    */
    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera1 = new PhotonCamera(kCameraName1);
        camera2 = new PhotonCamera(kCameraName2);
 
        photonEstimator1 =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam1);
        photonEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimator2 =
                new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam2);
        photonEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
    }

    @Override
    public void periodic() {
        //if (!DriverStation.isAutonomous()) {
            Optional<EstimatedRobotPose> visionEst1 = Optional.empty();
            for (var change : camera1.getAllUnreadResults()) {
                visionEst1 = photonEstimator1.update(change);
                updateEstimationStdDevs1(visionEst1, change.getTargets());            
                visionEst1.ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs1();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });

             }


            Optional<EstimatedRobotPose> visionEst2 = Optional.empty();
            for (var change : camera2.getAllUnreadResults()) {
                visionEst2 = photonEstimator2.update(change);
                updateEstimationStdDevs2(visionEst2, change.getTargets());            
                visionEst2.ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs2();
                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });

            }

            
        //}
    }

 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
    private void updateEstimationStdDevs1(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs1 = kCamera1SingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kCamera1SingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator1.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs1 = kCamera1SingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = kCamera1MultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs1 = estStdDevs;
             }
         }
     }
    
     private void updateEstimationStdDevs2(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs2 = kCamera2SingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kCamera2SingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator2.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs2 = kCamera2SingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = kCamera2MultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs2 = estStdDevs;
             }
         }
     }

    

     public List<PhotonPipelineResult> getResults(){
        return camera1.getAllUnreadResults();
     }

     public List<PhotonPipelineResult> getResults2(){
        return camera2.getAllUnreadResults();
     }

     public boolean isAimedAtTag() {
        var result = camera1.getLatestResult();
        if (result.hasTargets()) {
            // Check if centered. Tolerance 2 degrees?
            return Math.abs(result.getBestTarget().getYaw()) < 2.0;
        }
        return false;
     }

     public Optional<Double> getDistanceToTag(int tagId) {
        var result = camera1.getLatestResult();
        for (var target : result.getTargets()) {
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
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs1() {
         return curStdDevs1;
     }

     public Matrix<N3, N1> getEstimationStdDevs2(){
        return curStdDevs2;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         return visionSim.getDebugField();
     }
 
     @FunctionalInterface
     public static interface EstimateConsumer {
         public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
     }
 }