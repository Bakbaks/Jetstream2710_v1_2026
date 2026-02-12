package frc.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.Constants.Vision.*;
import frc.robot.Constants;

public class RotateToTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final int tagID;
    private final DoubleSupplier velocityX;
    private final DoubleSupplier velocityY;
    private final ProfiledPIDController rotationController;
    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    
    //private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
       // .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    // Allowed center goal tags
    private static final int[] ALLOWED_TAG_IDS = {10, 26};
    
    // Tuning constants
    private static final double kPTheta = 2.0;
    private static final double kMaxAngularRate = 4.5; // rad/s
    private static final double kMaxAngularAccel = 6.0; // rad/s²
    private static final double kRotationToleranceRadians = Math.toRadians(5); // Increased from 2° to allow rotation
    private int loopsAtGoal = 0;
    private static final int LOOPS_AT_GOAL_THRESHOLD = 10; // Must be at goal for 10 cycles

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
        
        addRequirements(drivetrain);
        // TODO: addRequirements(vision); - temporarily disabled to test if vision requirement blocks command
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
        
        // Calculate angle to face the tag from robot center (field frame)
        double angleToTag = Math.atan2(
            tagPose2d.getY() - robotPose.getY(),
            tagPose2d.getX() - robotPose.getX()
        );
        
        // Camera yaw from robot-to-camera transform (Z = yaw in radians).
        // Forward camera = 0; backward = Math.PI. Set in Constants.Vision.kRobotToCam.
        double cameraYawOffset = Constants.Vision.kRobotToCam.getRotation().getZ();
        double targetAngle = angleToTag + cameraYawOffset;
        
        // Normalize angle to [-π, π]
        while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;
        while (targetAngle < -Math.PI) targetAngle += 2 * Math.PI;
        
        System.out.println("[RotateToTag] Robot at: (" + robotPose.getX() + ", " + robotPose.getY() + ")" + "rotation : " + robotPose.getRotation());
        System.out.println("[RotateToTag] Tag " + tagID + " at: (" + tagPose2d.getX() + ", " + tagPose2d.getY() + ")");
        System.out.println("[RotateToTag] Angle to tag: " + Math.toDegrees(angleToTag) + "°");
        System.out.println("[RotateToTag] Camera yaw offset: " + Math.toDegrees(cameraYawOffset) + "°");
        System.out.println("[RotateToTag] Final target angle: " + Math.toDegrees(targetAngle) + "°");
        
        rotationController.setGoal(targetAngle);
        loopsAtGoal = 0; // Reset counter
        SmartDashboard.putString("RotateToTag/Status", "Initialized - targeting tag " + tagID);
        System.out.println("[RotateToTag] Goal set to: " + Math.toDegrees(targetAngle) + "°");
        System.out.println("[RotateToTag] PID Kp=" + rotationController.getP() + ", Kd=" + rotationController.getD());
        System.out.println("[RotateToTag] Waiting for rotation...");
        System.out.println("[RotateToTag] Keep holding right trigger to see robot rotate");
    }

    @Override
    public void execute() {
        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
        double rotationOutput = rotationController.calculate(currentAngle);
        
        SmartDashboard.putNumber("RotateToTag/CurrentAngle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber("RotateToTag/TargetAngle", Math.toDegrees(rotationController.getGoal().position));
        SmartDashboard.putNumber("RotateToTag/RotationOutput", rotationOutput);
        SmartDashboard.putBoolean("RotateToTag/AtGoal", rotationController.atGoal());
        SmartDashboard.putNumber("RotateToTag/ErrorDegrees", Math.toDegrees(rotationController.getPositionError()));
        
        // Debug: Print frequently to see what's happening
        System.out.printf("[RotateToTag EXEC] Angle: %.1f° Target: %.1f° Error: %.1f° Output: %.4f%n",
            Math.toDegrees(currentAngle),
            Math.toDegrees(rotationController.getGoal().position),
            Math.toDegrees(rotationController.getPositionError()),
            rotationOutput);
        
        // Use ApplyRobotSpeeds like the working example code
        drivetrain.setOperatorPerspectiveForward(Rotation2d.kZero);
        drivetrain.setControl(applyFieldSpeeds
            .withSpeeds(new ChassisSpeeds(
                velocityX.getAsDouble(),
                velocityY.getAsDouble(),
                rotationOutput
            ))
        );
    }

    @Override
    public boolean isFinished() {
        boolean atGoal = rotationController.atGoal();
        
        if (atGoal) {
            loopsAtGoal++;
            if (loopsAtGoal >= LOOPS_AT_GOAL_THRESHOLD) {
                System.out.println("[RotateToTag] At goal after " + loopsAtGoal + " cycles");
                return true;
            }
        } else {
            loopsAtGoal = 0; // Reset if we leave goal
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[RotateToTag] Command ended - Interrupted: " + interrupted);
        SmartDashboard.putString("RotateToTag/Status", interrupted ? "Interrupted" : "Completed");
        drivetrain.stop();
    }
}
