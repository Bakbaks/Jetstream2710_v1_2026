package frc.robot.commands.feed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExampleSubsystem.ExampleSubsystem;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.rollers.Rollers;

/** An example command that uses an example subsystem. */
public class Feedfuel extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Hopper m_hopper;
  private final Rollers m_shooter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Feedfuel(Hopper hopper, Rollers shooter) {
    m_hopper = hopper;
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_hopper);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.isVelocityWithinTolerance()) {
		m_hopper.setFloorRPM();
        m_hopper.setFeederRPM();
	} else {
		m_hopper.stop();
	}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}