package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;

    }

    public static class VisionConstants {
      public static final String kCameraName = "dave";

      /** Camera height above ground (meters). */
      public static final double kCameraHeightMeters = 1.0;
      /** AprilTag 10 center height above ground (meters). 44.25 in = 1.12395 m. */
      public static final double kTag10HeightMeters = Units.inchesToMeters(44.25);
      /** Camera pitch from horizontal in radians. 15 deg up from vertical = 75 deg from horizontal. */
      public static final double kCameraPitchRadians = Units.degreesToRadians(75);

      // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#coordinate-systems
      // Rotation3d(roll, pitch, yaw) in radians. Use (0,0,0) for forward camera, (0,0,Math.PI) for backward.
      public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, Math.PI));

      // The layout of the AprilTags on the field
      public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class FlywheelConstants {
      public static final int FlywheelStatorCurrentLimit = 120;
      public static final int FlywheelSupplyCurrentLimit = 70;
      
      
      public static final double kMinTargetRPM = 100; 
      public static final double kVelocityTolerance = 200;
      public static final double KFlywheelP = 0.5;
      public static final double KFlywheelI = 2;
      public static final double KFlywheelD = 0;

    }

    public static class ElevatorConstants {
      public static final double kElevatorPercent = 0.8;
      public static final int kElevatorCurrentLimit = 90;
      
      public static final double kElevatorMinOutput = -1;
      public static final double kElevatorMaxOutput = 1;
    }

    public static class HopperConstants {
      public static final double kFloorPercent = 0.5;
      public static final double kFeederPercent = 1;

      //no current limits cuz sped
      public static final int kHopperStatorCurrentLimit = 100;
      public static final int kHopperSupplyCurrentLimit = 70;
    
      
      public static final double kFloorRPM = 2000;
      public static final double kFeederRPM = 4000;
      public static final double KHopperP = 0.5;
      public static final double KHopperI = 2;
      public static final double KHopperD = 0;

     
    }

    public static class FieldConstants {

      public static final int RED_SHOOT_TAG = 10;
      public static final int BLUE_SHOOT_TAG = 26;
      //tags 10(red) & 26(blue)
      public static final Transform2d RightTagToHub = new Transform2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)); // needs to be measured still
    }

    public static class AutoConstants {
      
      // Tuning constants
      public static final double kPTheta = 2.0;
      public static final double kMaxAngularRate = 4.5; // rad/s
      public static final double kMaxAngularAccel = 6.0; // rad/s²
      public static final double kRotationToleranceRadians = Math.toRadians(5); // Increased from 2° to allow rotation
      public static final double kPositionToleranceMeters = 0.1;
    }

    public static class KrakenX60 {
      public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
  }