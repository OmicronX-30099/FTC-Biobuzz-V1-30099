package org.firstinspires.ftc.teamcode.control;

/** Linear interpolation of measured shot flight times, with endpoint hold outside calibration. */
public final class FlightTimeInterpolator {
    private FlightTimeInterpolator() { }

    // Averages from the recorded three-shot trials; distances sorted ascending for lookup.
    private static final double[] DISTANCE_IN = {23, 28, 38, 48, 58, 68};
    private static final double[] TIME_SECONDS = {
            0.473333333, 0.54, 0.436666667, 0.446666667, 0.5, 0.546666667
    };

    public static double atDistance(double distanceInches) {
        if (!Double.isFinite(distanceInches)) return Double.NaN;
        if (distanceInches <= DISTANCE_IN[0]) return TIME_SECONDS[0];
        if (distanceInches <= DISTANCE_IN[1]) return interpolate(distanceInches, 0, 1);
        if (distanceInches <= DISTANCE_IN[2]) return interpolate(distanceInches, 1, 2);
        if (distanceInches <= DISTANCE_IN[3]) return interpolate(distanceInches, 2, 3);
        if (distanceInches <= DISTANCE_IN[4]) return interpolate(distanceInches, 3, 4);
        if (distanceInches < DISTANCE_IN[5]) return interpolate(distanceInches, 4, 5);
        return TIME_SECONDS[5];
    }

    private static double interpolate(double distance, int low, int high) {
        double fraction = (distance - DISTANCE_IN[low]) /
                (DISTANCE_IN[high] - DISTANCE_IN[low]);
        return TIME_SECONDS[low] + fraction * (TIME_SECONDS[high] - TIME_SECONDS[low]);
    }
}
