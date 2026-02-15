package frc.robot.commands.shoot;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.util.FlywheelInterpolation;
import frc.robot.util.RobotLocalization;

import edu.wpi.first.wpilibj.DriverStation;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class Volley extends Command {
  private final Flywheel m_flywheel;
  private final Hopper m_hopper;
  private final Supplier<Pose2d> robotPoseSupplier;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public Volley(Flywheel flywheel, Hopper hopper, Supplier<Pose2d> robotPoseSupplier) {
    m_flywheel = flywheel;
    m_hopper = hopper;
    this.robotPoseSupplier = robotPoseSupplier;
    addRequirements(m_flywheel, m_hopper);
  }

  private int getAllianceTagId() {
  return DriverStation.getAlliance()
      .map(alliance ->
          alliance == DriverStation.Alliance.Red
              ? FieldConstants.RED_SHOOT_TAG
              : FieldConstants.BLUE_SHOOT_TAG)
      .orElse(FieldConstants.RED_SHOOT_TAG);
}


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    // Robot Pose to Goal distance
    Pose2d robotPose = robotPoseSupplier.get();
    int tagId = getAllianceTagId();
  
    Optional<Pose2d> maybeTargetPose =
        RobotLocalization.fieldPoseFromTagTransform(tagId, FieldConstants.RightTagToHub);
    if (maybeTargetPose.isEmpty()) {
      m_flywheel.stop();
      m_hopper.stop();
      return;
    }
    Pose2d targetPose = maybeTargetPose.get();

    double targetDistance = RobotLocalization.robotToTargetDistanceMeters(robotPose, targetPose);

    //Distance to RPM map
    double rpm = FlywheelInterpolation.getRPMForDistance(Optional.of(targetDistance));

    //Control
    m_flywheel.setRPM(rpm);

    if (m_flywheel.isVelocityWithinTolerance()) {
      m_hopper.setFloorRPM();
          m_hopper.setFeederRPM();
    } else {
      m_hopper.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.stop();
    m_hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}