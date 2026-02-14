package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Rollers;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FlywheelInterpolation;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class PopNAwe extends Command {
  private final Rollers m_rollers;
  private final Vision m_vision;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public PopNAwe(Rollers rollers, Vision vision) {
    m_rollers = rollers;
    m_vision = vision;
    addRequirements(m_rollers);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double rpm = FlywheelInterpolation.getRPMForDistance(m_vision.getDistanceToTag10());
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