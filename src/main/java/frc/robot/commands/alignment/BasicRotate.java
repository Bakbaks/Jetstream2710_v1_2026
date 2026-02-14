package frc.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.Constants.Vision.*;
import frc.robot.Constants;

public class BasicRotate extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final CommandXboxController controller;

    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    
    
    // Calculate drivetrain commands from Joystick values
   
    private double forward;
    private double strafe;
    private double turn;

    private boolean targetVisible = false;
    private double targetYaw = 0.0;

    private double KPtheta = 1.0;
    private double maxLinSpeed;
    private double maxAngSpeed;
    public BasicRotate(CommandSwerveDrivetrain drivetrain, Vision vision, CommandXboxController m_driverController, double maxLinSpeed, double maxAngSpeed) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.controller = m_driverController;
        
        
        forward = -m_driverController.getLeftY() * maxLinSpeed;
        strafe = -m_driverController.getLeftX() * maxLinSpeed;
        turn = -m_driverController.getRightX() * maxAngSpeed;
        this.maxAngSpeed = maxAngSpeed;
        this.maxLinSpeed = maxLinSpeed;

        // Read in relevant data from the Camera
        targetVisible = false;
        targetYaw = 0.0;
        addRequirements(drivetrain);
    }


    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        forward = -controller.getLeftY() * maxLinSpeed;
        strafe = -controller.getLeftX() * maxLinSpeed;

        var results = vision.getResults();
        
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 10) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align when requested
        if (targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = -1.0 * Math.toRadians(targetYaw) * KPtheta * maxAngSpeed; //kp turnin 2
            SmartDashboard.putNumber("Turn value", turn);
        }


        
        

        drivetrain.setControl(applyFieldSpeeds
            .withSpeeds(new ChassisSpeeds(
                forward,
                strafe,
                turn
            ))
        
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[RotateToTag] Command ended - Interrupted: " + interrupted);
        SmartDashboard.putString("RotateToTag/Status", interrupted ? "Interrupted" : "Completed");
        drivetrain.stop();
    }
}
