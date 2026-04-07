package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Optional;
import frc.robot.Constants.FlywheelConstants;

public final class FlywheelInterpolation {

    /** Default RPM when distance cannot be computed. */
    private static final double kDefaultRPM = FlywheelConstants.kDefaultRPM;

    /** Temporary fallback TOF if needed. Tune later. */
    private static final double kDefaultTOF = 0.28;

    /** Distance (m) -> shooter RPM */
    private static final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

    /** Distance (m) -> time of flight (s) */
    private static final InterpolatingDoubleTreeMap tofTable = new InterpolatingDoubleTreeMap();

    static {
        shooterTable.put(1.7, 1250.0);
        shooterTable.put(3.0, 1450.0);
        shooterTable.put(3.6, 1570.0);
        shooterTable.put(4.3, 1650.0);

        tofTable.put(1.7, 0.20);
        tofTable.put(3.0, 0.27);
        tofTable.put(3.6, 0.32);
        tofTable.put(4.3, 0.38);
    }

    private FlywheelInterpolation() {}

    public static double interpolateRPM(double distanceMeters) {
        return shooterTable.get(distanceMeters);
    }

    public static double interpolateTOF(double distanceMeters) {
        return tofTable.get(distanceMeters);
    }

    public static double getRPMForDistance(Optional<Double> distanceMeters) {
        return distanceMeters.map(FlywheelInterpolation::interpolateRPM).orElse(kDefaultRPM);
    }

    public static double getTOFForDistance(Optional<Double> distanceMeters) {
        return distanceMeters.map(FlywheelInterpolation::interpolateTOF).orElse(kDefaultTOF);
    }
}