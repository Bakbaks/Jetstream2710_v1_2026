package frc.robot.commands.alignment;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotLocalization;

public class ScoreOrientation extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;
    private final DoubleSupplier velocityW;

    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    private final ProfiledPIDController rotationController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPTheta,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularRate, Constants.AutoConstants.kMaxAngularAccel));

    public ScoreOrientation(CommandSwerveDrivetrain drivetrain, DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier velocityW) {
        this.drivetrain = drivetrain;   
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.velocityW = velocityW;
        addRequirements(drivetrain);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Constants.AutoConstants.kRotationToleranceRadians);
  }

    private int getAllianceTagId() {
        return DriverStation.getAlliance()
            .map(alliance ->
            alliance == DriverStation.Alliance.Red
                ? FieldConstants.RED_SHOOT_TAG
                : FieldConstants.BLUE_SHOOT_TAG)
        .orElse(FieldConstants.RED_SHOOT_TAG);
    }
    @Override
    public void initialize() {
        // Start controller from current heading so it doesn't "jump"
        double thetaNow = drivetrain.getPose().getRotation().getRadians();
        rotationController.reset(thetaNow);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        int tagId = getAllianceTagId();

        // Same style as Volley: compute target pose from tag + known transform
        Optional<Pose2d> maybeTargetPose =
        RobotLocalization.fieldPoseFromTagTransform(tagId, FieldConstants.RightTagToHub);

        if (maybeTargetPose.isEmpty()) {
            // Fail-safe: no target => don't rotate automatically
            drivetrain.setControl(
            applyFieldSpeeds.withSpeeds(
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                velocityX.getAsDouble(),
                velocityY.getAsDouble(),
                velocityW.getAsDouble())));
            SmartDashboard.putBoolean("ScoreOrientation/HasTarget", false);
            return;
        }

        SmartDashboard.putBoolean("ScoreOrientation/HasTarget", true);

        Pose2d targetPose = maybeTargetPose.get();

        // Robot-relative bearing to target
        Rotation2d bearingRobotFrame = RobotLocalization.robotToTargetBearing(robotPose, targetPose);

        
        // desiredHeadingField = robotHeadingField + (robot-relative bearing)
        Rotation2d desiredHeadingField = robotPose.getRotation().plus(bearingRobotFrame);

        double thetaNow = robotPose.getRotation().getRadians();
        double thetaGoal = desiredHeadingField.getRadians();

        double omegaCmd = rotationController.calculate(thetaNow, thetaGoal);

        drivetrain.setControl(
            applyFieldSpeeds.withSpeeds(
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    velocityX.getAsDouble(),
                    velocityY.getAsDouble(),
                    omegaCmd)));

    
        SmartDashboard.putNumber("ScoreOrientation/tagId", tagId);
        SmartDashboard.putNumber("ScoreOrientation/thetaNowDeg", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("ScoreOrientation/thetaGoalDeg", desiredHeadingField.getDegrees());
        SmartDashboard.putNumber("ScoreOrientation/omegaCmd", omegaCmd);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop(); // REMOVE once it work
  }

}
