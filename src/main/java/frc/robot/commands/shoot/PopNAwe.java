package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem.ExampleSubsystem;

import frc.robot.subsystems.shooter.Rollers;

/** An example command that uses an example subsystem. */
public class PopNAwe extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Rollers m_rollers;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PopNAwe(Rollers rollers) {
    m_rollers = rollers;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollers.setRPM(1000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}