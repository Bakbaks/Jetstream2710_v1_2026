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

    public static class Vision {
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

    public static class KrakenX60 {
      public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
  }