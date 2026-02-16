package frc.robot.commands.intake;

import java.net.InetSocketAddress;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Intake;
import frc.robot.util.FlywheelInterpolation;
import frc.robot.util.RobotLocalization;

import edu.wpi.first.wpilibj.DriverStation;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class HungryHungryHippo extends Command {
  private final Intake m_intake;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public HungryHungryHippo(Intake intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }




  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    // Robot Pose to Goal distance
    
    m_intake.setWheelRPM();
}
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}