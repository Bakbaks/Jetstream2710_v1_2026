package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem.ExampleSubsystem;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.hopper.Hopper;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Rollers m_rollers;
  private final Hopper m_hopper;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Rollers rollers, Hopper hopper) {
    m_rollers = rollers;
    m_hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rollers, m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollers.dashboardSpinUpCommand();
    if (m_rollers.isVelocityWithinTolerance()) {
      m_hopper.setFloorRPM();
          m_hopper.setFeederRPM();
    } else {
      m_hopper.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollers.stop();
    m_hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}