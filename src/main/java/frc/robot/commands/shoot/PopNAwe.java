package frc.robot.commands.shoot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Rollers;
import frc.robot.util.FlywheelInterpolation;

/** Shoots notes with flywheel speed interpolated from distance to target tag. */
public class PopNAwe extends Command {
  private final Rollers m_rollers;
  private final CommandSwerveDrivetrain m_drivetrain;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param drivetrain Drivetrain for robot pose (used for distance-based RPM)
   */
  public PopNAwe(Rollers rollers, CommandSwerveDrivetrain drivetrain) {
    m_rollers = rollers;
    m_drivetrain = drivetrain;
    addRequirements(m_rollers);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPose = m_drivetrain.getState().Pose;
    double rpm = FlywheelInterpolation.getRPMForPose(robotPose);
    m_rollers.setRPM(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollers.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}