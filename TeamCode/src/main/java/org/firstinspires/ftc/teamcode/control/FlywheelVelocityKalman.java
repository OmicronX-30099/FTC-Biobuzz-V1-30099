package org.firstinspires.ftc.teamcode.control;

/** Allocation-free scalar Kalman filter for measured flywheel velocity. */
public final class FlywheelVelocityKalman {
    private double estimate;
    private double covariance;
    private boolean initialized;

    /**
     * Noise inputs are standard deviations in ticks/second per control update.
     * Their squares are the process and measurement variances.
     */
    public double update(double measurement, double processNoiseStdDev,
                         double measurementNoiseStdDev) {
        if (!Double.isFinite(measurement)) return initialized ? estimate : 0.0;

        double processNoise = squareNonNegative(processNoiseStdDev, 20.0);
        double measurementNoise = squarePositive(measurementNoiseStdDev, 30.0);
        if (!initialized) {
            reset(measurement, measurementNoise);
            return estimate;
        }

        double predictedCovariance = covariance + processNoise;
        double gain = predictedCovariance / (predictedCovariance + measurementNoise);
        estimate += gain * (measurement - estimate);
        covariance = (1.0 - gain) * predictedCovariance;
        return estimate;
    }

    /** Seeds the filter from the live encoder reading when the shooter initializes. */
    public void reset(double initialMeasurement, double initialCovariance) {
        if (!Double.isFinite(initialMeasurement)) {
            initialized = false;
            estimate = 0.0;
            covariance = 0.0;
            return;
        }
        estimate = initialMeasurement;
        covariance = Double.isFinite(initialCovariance) && initialCovariance > 0.0
                ? initialCovariance : 900.0;
        initialized = true;
    }

    private static double squareNonNegative(double value, double fallback) {
        double safe = Double.isFinite(value) && value >= 0.0 ? value : fallback;
        return safe * safe;
    }

    private static double squarePositive(double value, double fallback) {
        double safe = Double.isFinite(value) && value > 0.0 ? value : fallback;
        return safe * safe;
    }
}
