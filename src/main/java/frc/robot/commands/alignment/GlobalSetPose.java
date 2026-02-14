package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

import java.util.Optional;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import static frc.robot.Constants.VisionConstants.*;


/** An example command that uses an example subsystem. */
public class GlobalSetPose extends Command {
    @SuppressWarnings("PMD.UnusedPrivateField")

    

    private final CommandSwerveDrivetrain m_Drivetrain;
    //going back facing the tag is negative x
    //going left facing the tag is positive y
    //rotating left is positive. invert tag orientation when going from tag to goal
    private final Pose2d goalFieldPose; // goal in field pse

    // Tolerances
    private static final double kTranslationToleranceMeters = 0.02; // 5 cm
    private static final double kRotationToleranceRadians = Math.toRadians(2); // 3 deg

    // Drive train settings and commands
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 2.5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //linear PID settings
    private static double kp = 0.5;
    private static double ki = 0.05;
    private static double kd = 0.03;
    
    // Translation PID controllers (field-frame X and Y)
    private final PIDController xController = new PIDController(kp, ki, kd);
    private final PIDController yController = new PIDController(kp, ki, kd);

    // Rotation P gain (you called it w; use a clearer name)
    private static final double kPTheta = 2.0; // rad/s per rad error (tune)

    /*
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
    */
    public GlobalSetPose(CommandSwerveDrivetrain m_Drivetrain, Pose2d goalFieldPose) {
        this.m_Drivetrain = m_Drivetrain;
        this.goalFieldPose = goalFieldPose;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.m_Drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //turn on led lights        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // determine error in goalFieldPose and PID controller to move robot there.
        //apply requests

        Pose2d curPose = m_Drivetrain.getPose();
    
        // Field frame error
        double ex = goalFieldPose.getX() - curPose.getX(); // meters
        double ey = goalFieldPose.getY() - curPose.getY(); // meters
    
        
        // PIDController.calculate(measurement, setpoint)
        double vxField = xController.calculate(curPose.getX(), goalFieldPose.getX());
        double vyField = yController.calculate(curPose.getY(), goalFieldPose.getY());
    
        // Clamp to max speed
        vxField = MathUtil.clamp(vxField, -MaxSpeed, MaxSpeed);
        vyField = MathUtil.clamp(vyField, -MaxSpeed, MaxSpeed);
    
        // Optional: if you want to stop commanding tiny velocities inside translation tolerance
        if (Math.hypot(ex, ey) < kTranslationToleranceMeters) {
            vxField = 0.0;
            vyField = 0.0;
        }
    
        //rotation
        double thetaErr = goalFieldPose.getRotation().minus(curPose.getRotation()).getRadians();
        double omega = kPTheta * thetaErr;
    
        omega = MathUtil.clamp(omega, -MaxAngularRate, MaxAngularRate);
    
        // Optional: deadband omega inside tolerance
        if (Math.abs(thetaErr) < kRotationToleranceRadians) {
            omega = 0.0;
        }
    
        //Apply request FieldCentric with field-relative velocities
        m_Drivetrain.setControl(
            drive.withVelocityX(vxField)
                 .withVelocityY(vyField)
                 .withRotationalRate(omega)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //leds

        //stop drivetrain.
        m_Drivetrain.setControl(brake);
    }
  
    @Override
    public boolean isFinished() {
        Pose2d curPose = m_Drivetrain.getPose();

        double translationError = curPose.getTranslation().getDistance(goalFieldPose.getTranslation());

        double rotationError = curPose.getRotation().minus(goalFieldPose.getRotation()).getRadians();

        return translationError < kTranslationToleranceMeters && Math.abs(rotationError) < kRotationToleranceRadians;
    }
  }