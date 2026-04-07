package frc.robot.commands.shoot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.util.FlywheelInterpolation;
import frc.robot.util.RobotLocalization;
import frc.robot.util.ShootOnMoveUtil;

public class Volley2 extends Command {
  private final Flywheel m_flywheel;
  private final Hopper m_hopper;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;
  private final BooleanSupplier constSpeedSupplier;

  public Volley2(
      Flywheel flywheel,
      Hopper hopper,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
      BooleanSupplier constSpeedSupplier) {
    this.m_flywheel = flywheel;
    this.m_hopper = hopper;
    this.robotPoseSupplier = robotPoseSupplier;
    this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;
    this.constSpeedSupplier = constSpeedSupplier;

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
  public void execute() {
    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldRelativeSpeedsSupplier.get();
    int tagId = getAllianceTagId();

    Optional<Pose2d> maybeTargetPose =
        RobotLocalization.fieldPoseFromTagTransform(tagId, FieldConstants.RightTagToHub);

    if (maybeTargetPose.isEmpty()) {
      m_flywheel.stop();
      m_hopper.stop();
      SmartDashboard.putBoolean("Volley2/HasTarget", false);
      return;
    }

    SmartDashboard.putBoolean("Volley2/HasTarget", true);

    Pose2d targetPose = maybeTargetPose.get();

    var solution = ShootOnMoveUtil.solve(
        robotPose,
        fieldSpeeds,
        targetPose.getTranslation()
    );

    double rpmCmd = FlywheelInterpolation.interpolateRPM(solution.effectiveDistance);

    SmartDashboard.putNumber("Volley2/RawDistance", solution.rawDistance);
    SmartDashboard.putNumber("Volley2/EffectiveDistance", solution.effectiveDistance);
    SmartDashboard.putNumber("Volley2/TOF", solution.timeOfFlight);
    SmartDashboard.putNumber("Volley2/RPMCmd", rpmCmd);
    SmartDashboard.putNumber("Volley2/PredictedExitX", solution.predictedExitPoint.getX());
    SmartDashboard.putNumber("Volley2/PredictedExitY", solution.predictedExitPoint.getY());

    if (constSpeedSupplier.getAsBoolean()) {
      m_flywheel.setRPM(FlywheelConstants.kDefaultRPM);
    } else {
      m_flywheel.setRPM(rpmCmd);
    }

    // For now keep feed behavior same as old Volley.
    // Next refinement is gating this on heading + flywheel readiness.
    m_hopper.setFloorRPM();
    m_hopper.setFeederRPM();
  }

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