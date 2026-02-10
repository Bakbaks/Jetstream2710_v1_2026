package frc.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.Constants.Vision.*;

public class RotateToTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final int tagID;
    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;
    private final ProfiledPIDController rotationController;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    // Allowed center goal tags
    private static final int[] ALLOWED_TAG_IDS = {10, 26};
    
    // Tuning constants
    private static final double kPTheta = 2.0;
    private static final double kMaxAngularRate = 4.5; // rad/s
    private static final double kMaxAngularAccel = 6.0; // rad/s²
    private static final double kRotationToleranceRadians = Math.toRadians(2);

    public RotateToTag(CommandSwerveDrivetrain drivetrain, Vision vision, int tagID, DoubleSupplier velocityX, DoubleSupplier velocityY) {
        // Validate tag ID
        if (!isAllowedTag(tagID)) {
            throw new IllegalArgumentException("Tag ID " + tagID + " not allowed. Use " + java.util.Arrays.toString(ALLOWED_TAG_IDS));
        }
        
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagID = tagID;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        
        this.rotationController = new ProfiledPIDController(
            kPTheta, 0, 0.1,
            new TrapezoidProfile.Constraints(kMaxAngularRate, kMaxAngularAccel)
        );
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationController.setTolerance(kRotationToleranceRadians);
        
        addRequirements(drivetrain, vision);
    }

    private static boolean isAllowedTag(int tagID) {
        for (int allowed : ALLOWED_TAG_IDS) {
            if (tagID == allowed) return true;
        }
        return false;
    }

    @Override
    public void initialize() {
        System.out.println("[RotateToTag] Initializing - attempting to locate tag ID: " + tagID);
        
        Optional<Pose3d> tagPose = kTagLayout.getTagPose(tagID);
        if (tagPose.isEmpty()) {
            System.err.println("[RotateToTag] ERROR: Tag ID " + tagID + " not found in field layout!");
            SmartDashboard.putString("RotateToTag/Error", "Tag " + tagID + " not in layout");
            cancel();
            return;
        }
        
        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d tagPose2d = tagPose.get().toPose2d();
        
        // Calculate angle to face the tag
        double targetAngle = Math.atan2(
            tagPose2d.getY() - robotPose.getY(),
            tagPose2d.getX() - robotPose.getX()
        );
        
        System.out.println("[RotateToTag] Robot at: (" + robotPose.getX() + ", " + robotPose.getY() + ")");
        System.out.println("[RotateToTag] Tag " + tagID + " at: (" + tagPose2d.getX() + ", " + tagPose2d.getY() + ")");
        System.out.println("[RotateToTag] Target angle: " + Math.toDegrees(targetAngle) + "°");
        
        rotationController.setGoal(targetAngle);
        SmartDashboard.putString("RotateToTag/Status", "Initialized - targeting tag " + tagID);
    }

    @Override
    public void execute() {
        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
        double rotationOutput = rotationController.calculate(currentAngle);
        
        SmartDashboard.putNumber("RotateToTag/CurrentAngle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("RotateToTag/TargetAngle", Math.toDegrees(rotationController.getGoal().position));
        SmartDashboard.putNumber("RotateToTag/RotationOutput", rotationOutput);
        SmartDashboard.putBoolean("RotateToTag/AtGoal", rotationController.atGoal());
        
        drivetrain.applyRequest(() -> drive
            .withVelocityX(velocityX.getAsDouble())
            .withVelocityY(velocityY.getAsDouble())
            .withRotationalRate(rotationOutput)
        );
    }

    @Override
    public boolean isFinished() {
        return rotationController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[RotateToTag] Command ended - Interrupted: " + interrupted);
        SmartDashboard.putString("RotateToTag/Status", interrupted ? "Interrupted" : "Completed");
        
        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }
}
