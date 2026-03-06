package frc.robot.commands.intake;

import java.net.InetSocketAddress;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.Intake.IntakeRollers.Speed;
import frc.robot.subsystems.Hopper;
import frc.robot.util.FlywheelInterpolation;
import frc.robot.util.RobotLocalization;

import edu.wpi.first.wpilibj.DriverStation;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class ReverseRollers extends Command {
  private final IntakeRollers m_intakeRollers;
  private final Hopper m_hopper;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public ReverseRollers(IntakeRollers intakeRollers, Hopper hopper) {
    m_intakeRollers = intakeRollers;
    m_hopper = hopper;
    addRequirements(m_intakeRollers, m_hopper);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    // Robot Pose to Goal distance
    
    m_intakeRollers.setIntakeSpeed(Speed.OUTAKE);
    //m_hopper.setFloorRPM();
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeRollers.setIntakeSpeed(Speed.STOP);
    //m_hopper.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}