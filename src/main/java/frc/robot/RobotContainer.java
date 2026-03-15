// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

//utils
import java.util.function.DoubleSupplier;

//pheonix6
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

//math
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.OutTake;
import frc.robot.commands.alignment.ScoreOrientation;
import frc.robot.commands.intake.DebugDetractIntake;
import frc.robot.commands.intake.DebugExtendIntake;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.DetractIntake;


//temp commands
import frc.robot.commands.intake.SpinRollers;
import frc.robot.commands.intake.ReverseRollers;
import frc.robot.commands.shoot.Volley;
import frc.robot.commands.shoot.TempVolley;
import frc.robot.commands.intake.SpinFloor;


import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ExampleSubsystem.ExampleSubsystem;
import frc.robot.subsystems.Intake.IntakeRollers;
import frc.robot.subsystems.Intake.IntakeExtendo;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.Vision;


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

  private final CommandXboxController m_auxController =
      new CommandXboxController(OperatorConstants.kAuxControllerPort);

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

  private final Trigger auxY = m_auxController.y();
  private final Trigger auxA = m_auxController.a();
  private final Trigger auxB = m_auxController.b();
  private final Trigger auxX = m_auxController.x();
  private final Trigger auxRightTrigger = m_auxController.rightTrigger();
  private final Trigger auxLeftTrigger  = m_auxController.leftTrigger();
  private final Trigger auxLeftBumper = m_auxController.leftBumper();
  private final Trigger auxRightBumper = m_auxController.rightBumper();
  private final Trigger auxPovDOWN = m_auxController.povDown();

  private final Pose2d goalpose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  private final Flywheel flywheel = new Flywheel();
  private final Hopper hopper = new Hopper();
  private final IntakeRollers intakeRollers = new IntakeRollers();
  private final IntakeExtendo intakeExtendo = new IntakeExtendo();
  private final Telemetry telemetry = new Telemetry(MaxSpeed, vision, flywheel, hopper, intakeRollers, intakeExtendo);
  private Boolean ConstSpeed = false;
  private boolean driverIntakeExtended = false;

  //Autos
  PathConstraints lims = new PathConstraints(
    4.4,                     // max m/s
    3.500,                     // max m/s^2
    Math.toRadians(540.0),   // max rad/s
    Math.toRadians(720.0)    // max rad/s^2

  );


  private final SendableChooser<Command> autoChooser;

  Command PreloadVolley = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new TempVolley(flywheel, hopper, intakeRollers, drivetrain::getPose, 1325)
      ).withTimeout(2)
  );

  Command ExtendIntake = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new DebugExtendIntake(intakeExtendo),
      new SpinRollers(intakeRollers)
    ).withTimeout(2)
  );

  Command ExtraVolley = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new TempVolley(flywheel, hopper, intakeRollers, drivetrain::getPose, 1325)
      ).withTimeout(5)
  );

  Command RetractIntake = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new SequentialCommandGroup(
        new DebugDetractIntake(intakeExtendo).withTimeout(0.35),
        new DebugExtendIntake(intakeExtendo).withTimeout(0.25),
        new DebugDetractIntake(intakeExtendo).withTimeout(0.35),
        new DebugExtendIntake(intakeExtendo).withTimeout(0.25),
        new DebugDetractIntake(intakeExtendo).withTimeout(0.35)
      )
    ).withTimeout(3)
  );
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //AutoCommands

    
    
    NamedCommands.registerCommand("PreloadVolley", PreloadVolley);
    NamedCommands.registerCommand("ExtendIntake", ExtendIntake);
    NamedCommands.registerCommand("ExtraVolley", ExtraVolley);
    NamedCommands.registerCommand("RetractIntake", RetractIntake);

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
    //

    // Driver Control binding
    // sticks are normal
    // RTrig is Shoot & spin up and score - works fine
    // RBump is Aim
    // LBump is hopper out toggle
    // LTrigger is intake spinning

    // Remove vision if it somehow causes issues: A ~~~~~~~~~~~~~~~~~~
    //Re Zero d pad down

    // Aux Control Debug binding
    // X is outake
    // Manual Intake movement Right and Left Triggger in and out
    // Disable pose: A ~~~~~~~~~~~~~~~~~~~~~~~~
    // Change shooter velocity to constant if interpolation is not work: B
    // Rezero intake d pad down


    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive
        .withVelocityX(-MathProfiles.exponentialDrive(m_driverController.getLeftY(), 3) * MaxSpeed)
        .withVelocityY(-MathProfiles.exponentialDrive(m_driverController.getLeftX(), 3) * MaxSpeed)
        .withRotationalRate(-MathProfiles.exponentialDrive(m_driverController.getRightX(), 2) * MaxAngularRate))
    );
    drivePovDOWN.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    //hopper.setDefaultCommand(hopper.floorSiftRPMCommand());

    DoubleSupplier aimVX =
        () -> -MathProfiles.exponentialDrive(m_driverController.getLeftY(), 3) * MaxSpeed;

    DoubleSupplier aimVY =
        () -> -MathProfiles.exponentialDrive(m_driverController.getLeftX(), 3) * MaxSpeed;

    DoubleSupplier rotV =
        () -> -MathProfiles.exponentialDrive(m_driverController.getRightX(), 2) * MaxAngularRate;

    driveRightTrigger.whileTrue(new ParallelCommandGroup(
      new Volley(flywheel, hopper, intakeRollers, drivetrain::getPose, ConstSpeed)
    ));
 
    driveRightBumper.whileTrue(new ParallelCommandGroup(
      new ScoreOrientation(drivetrain, aimVX, aimVY, rotV)  // Timeout after 5 seconds to prevent hanging
    )); 

    // //private boolean driverIntakeExtended = false;
    // driveLeftBumper.onTrue(
    // Commands.runOnce(() -> {
    //   if (!driverIntakeExtended) {
    //     new ExtendIntake(intakeExtendo);
    //     driverIntakeExtended = true;
    //   } else {
    //     new DetractIntake(intakeExtendo);
    //     driverIntakeExtended = false;
    //   }
    // })
    // );
    

    driveLeftTrigger.whileTrue(new ParallelCommandGroup( // can on true if u want
      new SpinRollers(intakeRollers)
    ));

    driveLeftBumper.whileTrue(new ParallelCommandGroup(
      new OutTake(flywheel, hopper, intakeRollers)
    ));



    auxRightTrigger.whileTrue(new ParallelCommandGroup(
      new DebugDetractIntake(intakeExtendo)
    ));

    auxLeftTrigger.whileTrue(new ParallelCommandGroup(
      new DebugExtendIntake(intakeExtendo)
    ));

    auxRightBumper.whileTrue(new ParallelCommandGroup(
      new SpinFloor(hopper)
    ));

    // auxPovDOWN.onTrue(
    //   Commands.runOnce(() -> intakeExtendo.setExtendoZero())
    // );

    auxB.onTrue(
      Commands.runOnce(() -> {
        ConstSpeed = !ConstSpeed;
        SmartDashboard.putBoolean("ConstSpeed", ConstSpeed);
      })
    );
      

  }

  /** Updates SmartDashboard telemetry. Call from robotPeriodic. */
  public void updateTelemetry() {
    telemetry.telemeterize(drivetrain.getState());
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
