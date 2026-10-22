package org.firstinspires.ftc.teamcode.control;

/** Interpolates paired, empirically calibrated TPS/servo commands, not launch angles. */
public final class DistanceShotInterpolator {
    private DistanceShotInterpolator() { }

    public static final class Shot {
        public double velocityTPS = Double.NaN;
        public double hoodPosition = Double.NaN;
        public boolean clamped;

        /** Caller-owned reusable output; do not share between control threads. */
        public Shot() { }

        private void set(double velocityTPS, double hoodPosition, boolean clamped) {
            this.velocityTPS = velocityTPS;
            this.hoodPosition = hoodPosition;
            this.clamped = clamped;
        }
    }

    /** Invalid feedback or tables return null; out-of-range distances use the nearest endpoint. */
    public static Shot sample(double distance, double[] distances, double[] velocities, double[] hoods) {
        Shot result = new Shot();
        return sampleInto(distance, distances, velocities, hoods, result) ? result : null;
    }

    /** Allocation-free control-loop path. False invalidates the output; never reuse a stale shot. */
    public static boolean sampleInto(double distance, double[] distances, double[] velocities,
                                     double[] hoods, Shot result) {
        result.set(Double.NaN, Double.NaN, false);
        if (!Double.isFinite(distance) || distance < 0 || distances == null
                || velocities == null || hoods == null || distances.length < 2
                || distances.length != velocities.length || distances.length != hoods.length) return false;
        for (int i = 0; i < distances.length; i++) {
            if (!Double.isFinite(distances[i]) || distances[i] < 0
                    || (i > 0 && distances[i] <= distances[i - 1])
                    || !Double.isFinite(velocities[i]) || velocities[i] < 0
                    || !Double.isFinite(hoods[i]) || hoods[i] < 0 || hoods[i] > 1) return false;
        }
        int last = distances.length - 1;
        if (distance <= distances[0]) {
            result.set(velocities[0], hoods[0], distance < distances[0]);
            return true;
        }
        if (distance >= distances[last]) {
            result.set(velocities[last], hoods[last], distance > distances[last]);
            return true;
        }
        for (int i = 1; i <= last; i++) {
            if (distance <= distances[i]) {
                double fraction = (distance - distances[i - 1]) / (distances[i] - distances[i - 1]);
                result.set(velocities[i - 1] + fraction * (velocities[i] - velocities[i - 1]),
                        hoods[i - 1] + fraction * (hoods[i] - hoods[i - 1]), false);
                return true;
            }
        }
        return false;
    }
}
