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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Inches;
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
      public static final int kAuxControllerPort = 1;

    }

    public static class VisionConstants {
      public static final String kCameraName = "dave";

      /** AprilTag 10 center height above ground (meters). 44.25 in = 1.12395 m. */
      /** Camera pitch from horizontal in radians. 15 deg up from vertical = 75 deg from horizontal. */
      // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#coordinate-systems
      // Rotation3d(roll, pitch, yaw) in radians. Use (0,0,0) for forward camera, (0,0,Math.PI) for backward.
      public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(-0.2931456926, 0, 0.7338025964), new Rotation3d(0, Units.degreesToRadians(15), Math.PI));

      // The layout of the AprilTags on the field
      public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      /*
      *  Standard deviations of the vision pose measurement (x position
      *  in meters, y position in meters, and heading in radians). Increase 
      *  thes numbers to trustthe vision pose measurement less.
      **/
      // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.7, .7, 9999999);
      // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.7, .7, 9999999);
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.7, 0.7, 0.6);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.7, 0.7, 0.6);
    }

    public static class FlywheelConstants {
      public static final int FlywheelStatorCurrentLimit = 120;
      public static final int FlywheelSupplyCurrentLimit = 90;
      
      
      public static final double kMinTargetRPM = 100; 
      public static final double kVelocityTolerance = 800;
      public static final double KFlywheelP = 0.5;
      public static final double KFlywheelI = 2;
      public static final double KFlywheelD = 0;

      public static final double kDefaultRPM = 1350;

      public static final double kFlywheelReverseRPM = -900;
    }

    public static class ElevatorConstants {
      public static final double kElevatorPercent = 0.8;
      public static final int kElevatorCurrentLimit = 90;
      
      public static final double kElevatorMinOutput = -1;
      public static final double kElevatorMaxOutput = 1;
    }

    public static class HopperConstants {
      public static final double kFloorPercent = 0.6;
      public static final double kFeederPercent = 0.7;

      //no current limits cuz sped
      public static final int kHopperStatorCurrentLimit = 100;
      public static final int kHopperSupplyCurrentLimit = 100;
    
      
      public static final double kFloorRPM = 1500;
      public static final double kFeederRPM = 5000;
      public static final double KHopperP = 2;
      public static final double KHopperI = 0;
      public static final double KHopperD = 0;



      //sifting for gold
      public static final double kFloorSiftFrequencyHz = 1.0;
      public static final double kFloorSiftAmplitudeRPM = 350.0;
      public static final double kFloorSiftBiasRPM = 100.0;

      public static final double kFeederReverseRPM = -900;
      public static final double kFloorReverseRPM = -900;
    }

    public static class IntakeConstants {

      // every pi inches moved is 3 rotations.

      public static final double kExtendoPercent = 0.1;
      public static final double kRollerPercent = 0.8;
      
      public static final int kExtendoStatorCurrentLimit = 60;
      public static final int kExtendoSupplyCurrentLimit = 20;

      public static final int kRollerStatorCurrentLimit = 100;
      public static final int kRollerSupplyCurrentLimit = 80;
    
      //For the Extendo
      public static final double kMotorToPinionReduction = 5.0;
      public static final double kPinionRotPerPiInches = 3.0;

      public static final double kInchesPerPinionRotation = Math.PI / kPinionRotPerPiInches; // pi/3
      public static final double kInchesPerMotorRotation = kInchesPerPinionRotation / kMotorToPinionReduction; // pi/15
      public static final double kMotorRotationsPerInch = 1.0 / kInchesPerMotorRotation; // 15/pi

      public static final Distance kPositionTolerance = Inches.of(0.25);


      public static final double kExtendoIntakeP = 0.5;
      public static final double kExtendoIntakeI = 0;
      public static final double kExtendoIntakeD = 0;

      // Jiggle tuning (for retract / move-back shake)
      public static final double kExtendoJiggleFrequencyHz = 3.0;      // cycles per second
      public static final Distance kExtendoJiggleStep = Inches.of(0.25); // peak amplitude at start (in)
      public static final Distance kExtendoJiggleMinStep = Inches.of(0.05); // optional: don't go smaller than this until very close
      public static final Distance kExtendoJiggleHoldStep = Inches.of(0.10); // jiggle amplitude once "at goal"

      public static final double kIntakeReverseRPM = 400;
    }

    public static class FieldConstants {

      public static final int RED_SHOOT_TAG = 10;
      public static final int BLUE_SHOOT_TAG = 26;
      //tags 10(red) & 26(blue)
      public static final Transform2d RightTagToHub = new Transform2d(0.0, -0.737102412, Rotation2d.fromDegrees(180.0)); // needs to be measured still
    }

    public static class AutoConstants {
      // Tuning constants
      public static final double kPTheta = 2.0; // go here to tune
      public static final double kMaxAngularRate = 4.5; // rad/s
      public static final double kMaxAngularAccel = 6.0; // rad/s²
      public static final double kRotationToleranceRadians = Math.toRadians(5);
      public static final double kPositionToleranceMeters = 0.1;

      public static final double kRotOverrideThreshold = 0.5; // needs to be tuned based on driver preference
    }

    public static class KrakenX60 {
      public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
  }