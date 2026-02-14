// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//units

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

//pheonix6
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

//math
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
//wpilib
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ExampleSubsystem.ExampleSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.rollers.Rollers;

//Auto
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;


//robotcommands
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.alignment.TagSetPose;
import frc.robot.commands.alignment.GlobalSetPose;
import frc.robot.commands.alignment.DriveAutoLock;
import frc.robot.commands.alignment.RotateToTag;
import frc.robot.commands.ExampleCommand;

import frc.robot.commands.shoot.Shoot;
import frc.robot.commands.feed.Feedfuel;



import frc.robot.Constants.OperatorConstants;
import java.util.Optional;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //drivetrain
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  //Swerve Subsystem:
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            //.withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 2.5% deadband :: already built in deadbands so dont need
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  

  //private final Telemetry logger = new Telemetry(MaxSpeed);

  public final Vision vision = new Vision(drivetrain::addVisionMeasurement);


  // private final CommandXboxController m_driverController = new CommandXboxController(
  //           OIConstants.kDriverControllerPort);
  //   public static CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);
  //   public static XboxController m_Controller = new XboxController(OIConstants.kAuxControllerPort);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //controller sticks

  private final Trigger driveY = m_driverController.y();
  private final Trigger driveA = m_driverController.a();
  private final Trigger driveB = m_driverController.b();
  private final Trigger driveX = m_driverController.x();
  private final Trigger driveRightBumper = m_driverController.rightBumper();
  private final Trigger driveRightTrigger = m_driverController.rightTrigger();
  private final Trigger driveLeftBumper = m_driverController.leftBumper();
  private final Trigger driveLeftTrigger = m_driverController.leftTrigger();
  private final Trigger drivePovDOWN = m_driverController.povDown();
  private final Trigger drivePovUP = m_driverController.povUp();

  private final Pose2d goalpose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  private final Rollers rollers = new Rollers();
  private final Hopper hopper = new Hopper();

  //Autos ------------------------------------------------------------------------------------------------------------
  PathConstraints lims = new PathConstraints(
    3.0,                     // max m/s
    1.0,                     // max m/s^2
    Math.toRadians(540.0),   // max rad/s
    Math.toRadians(720.0)    // max rad/s^2
  );
  private final SendableChooser<Command> autoChooser;
  Command UnloadPreload = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new Shoot(rollers)
      ).withTimeout(2)
  );
  //------------------------------------------------------------------------------------------------------------------

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //AutoCommands

    
    
    NamedCommands.registerCommand("UnloadPrelaod", UnloadPreload);

    autoChooser = AutoBuilder.buildAutoChooser("Taxi");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
      
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //drive commands
    //set default command for drive
    //drivetrain.setDefaultCommand(new Drive(drivetrain, () -> m_driverController.getLeftY(), () -> m_driverController.getLeftX(), () -> m_driverController.getRightX()));
    
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-MathProfiles.exponentialDrive(m_driverController.getLeftY(), 3) * MaxSpeed)
                .withVelocityY(-MathProfiles.exponentialDrive(m_driverController.getLeftX(), 3) * MaxSpeed)
                .withRotationalRate(-MathProfiles.exponentialDrive(m_driverController.getRightX(), 2) * MaxAngularRate))
    );
    drivePovDOWN.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Shoot + rotate to face center goal tag while allowing translation
    driveRightTrigger.whileTrue(new ParallelCommandGroup(
      new Shoot(rollers),
      new RotateToTag(drivetrain, vision, 10,  // CHANGE TO BASED ON AUTO SELECTED LATER
        () -> -MathProfiles.exponentialDrive(m_driverController.getLeftY(), 3) * MaxSpeed,
        () -> -MathProfiles.exponentialDrive(m_driverController.getLeftX(), 3) * MaxSpeed
      ).withTimeout(5.0),  // Timeout after 5 seconds to prevent hanging
      // Run conveyor when shooter is up to speed
      new Feedfuel(hopper, rollers)
    ));

    //aux commands
      // make branch

    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
