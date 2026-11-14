package org.firstinspires.ftc.teamcode.control;

/** Polynomial flywheel TPS from goal distance in inches, with a calibrated hood lookup. */
public final class DistanceShotModel {
    private DistanceShotModel() { }

    public static double velocityTPS(double distanceInches) {
        if (!Double.isFinite(distanceInches) || distanceInches < 0) return Double.NaN;
        return ((-0.00962426 * distanceInches + 1.43626) * distanceInches
                - 62.22049) * distanceInches + 2295.8006;
    }

    public static DistanceShotInterpolator.Shot sample(double distance, double[] distances,
                                                       double[] hoods) {
        DistanceShotInterpolator.Shot result = new DistanceShotInterpolator.Shot();
        return sampleInto(distance, distances, hoods, result) ? result : null;
    }

    /** Only the hood holds its endpoint outside the table; velocity uses the actual distance. */
    public static boolean sampleInto(double distance, double[] distances, double[] hoods,
                                     DistanceShotInterpolator.Shot result) {
        result.velocityTPS = Double.NaN;
        result.hoodPosition = Double.NaN;
        result.clamped = false;
        double velocity = velocityTPS(distance);
        if (!Double.isFinite(velocity) || velocity < 0 || distances == null || hoods == null
                || distances.length < 2 || distances.length != hoods.length) return false;
        for (int i = 0; i < distances.length; i++) {
            if (!Double.isFinite(distances[i]) || distances[i] < 0
                    || (i > 0 && distances[i] <= distances[i - 1])
                    || !Double.isFinite(hoods[i]) || hoods[i] < 0 || hoods[i] > 1) return false;
        }
        int last = distances.length - 1;
        double hood = hoods[last];
        if (distance <= distances[0]) hood = hoods[0];
        else {
            for (int i = 1; i <= last; i++) {
                if (distance <= distances[i]) {
                    double fraction = (distance - distances[i - 1]) / (distances[i] - distances[i - 1]);
                    hood = hoods[i - 1] + fraction * (hoods[i] - hoods[i - 1]);
                    break;
                }
            }
        }
        result.velocityTPS = velocity;
        result.hoodPosition = hood;
        result.clamped = distance < distances[0] || distance > distances[last];
        return true;
    }
}
