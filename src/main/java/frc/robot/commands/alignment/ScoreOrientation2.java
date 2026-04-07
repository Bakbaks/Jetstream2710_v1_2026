package frc.robot.commands.alignment;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShotConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotLocalization;
import frc.robot.util.ShootOnMoveUtil;

public class ScoreOrientation2 extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;
    private final DoubleSupplier velocityW;

    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds =
        new SwerveRequest.ApplyFieldSpeeds();

    private final ProfiledPIDController rotationController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPTheta,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Constants.AutoConstants.kMaxAngularRate,
                Constants.AutoConstants.kMaxAngularAccel
            )
        );

    public ScoreOrientation2(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier velocityW) {
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

    private boolean shouldFlipForAlliance() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Blue)
            .orElse(false);
    }

    @Override
    public void initialize() {
        double thetaNow = drivetrain.getPose().getRotation().getRadians();
        rotationController.reset(thetaNow);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        int tagId = getAllianceTagId();

        Optional<Pose2d> maybeTargetPose =
            RobotLocalization.fieldPoseFromTagTransform(tagId, FieldConstants.RightTagToHub);

        double xSpeed = velocityX.getAsDouble();
        double ySpeed = velocityY.getAsDouble();

        if (shouldFlipForAlliance()) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }

        double maxSpeed = AutoConstants.MaxShootMoveSpeed;
        double clampedXSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
        double clampedYSpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);

        if (maybeTargetPose.isEmpty()) {
            drivetrain.setControl(
                applyFieldSpeeds.withSpeeds(
                    new edu.wpi.first.math.kinematics.ChassisSpeeds(
                        clampedXSpeed,
                        clampedYSpeed,
                        velocityW.getAsDouble()
                    )
                )
            );

            SmartDashboard.putBoolean("ScoreOrientation/AllianceFlip", shouldFlipForAlliance());
            SmartDashboard.putBoolean("ScoreOrientation/HasTarget", false);
            return;
        }

        SmartDashboard.putBoolean("ScoreOrientation/HasTarget", true);

        Pose2d targetPose = maybeTargetPose.get();

        // Use actual measured drivetrain motion, not joystick commands, for shot prediction
        var fieldSpeeds = drivetrain.getFieldRelativeSpeeds();

        var solution = ShootOnMoveUtil.solve(
            robotPose,
            fieldSpeeds,
            targetPose.getTranslation()
        );

        Rotation2d desiredHeadingField = solution.desiredHeadingField;

        double thetaNow = robotPose.getRotation().getRadians();
        double thetaGoal = desiredHeadingField.getRadians();

        double omegaCmd = rotationController.calculate(thetaNow, thetaGoal);

        drivetrain.setControl(
            applyFieldSpeeds.withSpeeds(
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    clampedXSpeed,
                    clampedYSpeed,
                    omegaCmd
                )
            )
        );

        double thetaError = MathUtil.angleModulus(thetaGoal - thetaNow);
        boolean canShoot = Math.abs(thetaError) < ShotConstants.kShootHeadingToleranceRad;

        SmartDashboard.putBoolean("ScoreOrientation/AllianceFlip", shouldFlipForAlliance());
        SmartDashboard.putNumber("ScoreOrientation/tagId", tagId);
        SmartDashboard.putNumber("ScoreOrientation/thetaNowDeg", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("ScoreOrientation/thetaGoalDeg", desiredHeadingField.getDegrees());
        SmartDashboard.putNumber("ScoreOrientation/thetaErrorDeg", Math.toDegrees(thetaError));
        SmartDashboard.putNumber("ScoreOrientation/omegaCmd", omegaCmd);

        SmartDashboard.putNumber("ScoreOrientation/RawDistance", solution.rawDistance);
        SmartDashboard.putNumber("ScoreOrientation/EffectiveDistance", solution.effectiveDistance);
        SmartDashboard.putNumber("ScoreOrientation/TOF", solution.timeOfFlight);
        SmartDashboard.putNumber("ScoreOrientation/PredictedExitX", solution.predictedExitPoint.getX());
        SmartDashboard.putNumber("ScoreOrientation/PredictedExitY", solution.predictedExitPoint.getY());
        SmartDashboard.putNumber("ScoreOrientation/ExitVelX", solution.shooterExitVelocityField.getX());
        SmartDashboard.putNumber("ScoreOrientation/ExitVelY", solution.shooterExitVelocityField.getY());

        SmartDashboard.putBoolean("CanShoot", canShoot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}