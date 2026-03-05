package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.Intake.IntakeExtendo;
import frc.robot.subsystems.Intake.IntakeExtendo.Position;
import frc.robot.subsystems.Intake.IntakeRollers.Speed;
import frc.robot.Constants.HopperConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Shoots notes with flywheel speed interpolated from PhotonVision distance to tag 10. */
public class OutTake extends Command {
  private final Flywheel m_flywheel;
  private final Hopper m_hopper;
  private final IntakeRollers m_intakeRollers;
  private final IntakeExtendo m_intakeExtendo;

  /**
   * Creates a PopNAwe command.
   *
   * @param rollers Shooter subsystem
   * @param vision Vision for PhotonVision distance to tag 10
   */
  public OutTake(Flywheel flywheel, Hopper hopper, IntakeRollers intakeRollers, IntakeExtendo intakeExtendo) {
    m_flywheel = flywheel;
    m_hopper = hopper;
    m_intakeRollers = intakeRollers;
    m_intakeExtendo = intakeExtendo;
    addRequirements(m_flywheel, m_hopper, m_intakeRollers, m_intakeExtendo);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    m_flywheel.setRPM(FlywheelConstants.kFlywheelReverseRPM);

    m_hopper.setFloorRPM(HopperConstants.kFloorReverseRPM);
    m_hopper.setFeederRPM(HopperConstants.kFeederReverseRPM);

    m_intakeExtendo.setExtendoPosition(Position.EXTENDED);
    m_intakeRollers.setIntakeSpeed(Speed.OUTAKE);
    //set intake reverse rpm and max position

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.stop();
    m_hopper.stop();
    m_intakeRollers.setIntakeSpeed(Speed.STOP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}