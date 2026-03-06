package frc.robot.commands.intake;

import java.net.InetSocketAddress;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Intake.IntakeExtendo;
import frc.robot.subsystems.Intake.IntakeExtendo.Position;
import frc.robot.util.FlywheelInterpolation;
import frc.robot.util.RobotLocalization;

import edu.wpi.first.wpilibj.DriverStation;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class DebugDetractIntake extends Command {
  private final IntakeExtendo m_intakeExtendo;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public DebugDetractIntake(IntakeExtendo intakeExtendo) {
    m_intakeExtendo = intakeExtendo;
    addRequirements(m_intakeExtendo);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

    m_intakeExtendo.setExtendoPercentOutput(-0.3);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeExtendo.setExtendoPercentOutput(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}