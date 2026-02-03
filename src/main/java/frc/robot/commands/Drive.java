package frc.robot.commands;

import frc.robot.MathProfiles;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ExampleSubsystem.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;

/** An example command that uses an example subsystem. */
public class Drive extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final CommandSwerveDrivetrain m_Drivetrain;

  // Drive train settings and commands
  private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 2.5% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  
  private DoubleSupplier  getLeftY;
  private DoubleSupplier  getLeftX;
  private DoubleSupplier  getRightX;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(CommandSwerveDrivetrain drivetrain, DoubleSupplier getLeftY, DoubleSupplier getLeftX, DoubleSupplier getRightX) {
    this.m_Drivetrain = drivetrain;
    this.getLeftY = getLeftY;
    this.getLeftX = getLeftX;
    this.getRightX = getRightX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //leds blinking 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //leds stable

    // m_Drivetrain.applyRequest(() -> drive
    //           .withVelocityX(-MathProfiles.exponentialDrive(getLeftY.getAsDouble(), 3) * MaxSpeed)
    //           .withVelocityY(-MathProfiles.exponentialDrive(getLeftX.getAsDouble(), 3) * MaxSpeed)
    //           .withRotationalRate(-MathProfiles.exponentialDrive(getRightX.getAsDouble(), 2) * MaxAngularRate));
    
    
    m_Drivetrain.setControl(
      fieldCentricRequest
      .withVelocityX(-MathProfiles.exponentialDrive(getLeftY.getAsDouble(), 3) * MaxSpeed)
      .withVelocityY(-MathProfiles.exponentialDrive(getLeftX.getAsDouble(), 3) * MaxSpeed)
      .withRotationalRate(-MathProfiles.exponentialDrive(getRightX.getAsDouble(), 2) * MaxAngularRate));
  
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}