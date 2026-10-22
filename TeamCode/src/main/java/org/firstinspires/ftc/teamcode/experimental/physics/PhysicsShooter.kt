package org.firstinspires.ftc.teamcode.experimental.physics

import kotlin.math.*

/** Caller-owned result; reuse one instance in the control loop. Diagnostics are LUT estimates. */
class PhysicsShot {
    var hoodAngleDegrees = Double.NaN
    var launchAngleDegrees = Double.NaN
    var shooterVelocityTPS = Double.NaN
    var entryAngleDegrees = Double.NaN
    var vertexHeightCm = Double.NaN
    var vertexDistanceInches = Double.NaN
    var flightTimeSeconds = Double.NaN
    var ballExitVelocityMps = Double.NaN
    var entryAngleErrorDegrees = Double.NaN
    var heightConstraintSatisfied = false
    var feasible = false
    var valid = false
    var status = "Not calculated"

    internal fun invalidate() {
        hoodAngleDegrees = Double.NaN
        launchAngleDegrees = Double.NaN
        shooterVelocityTPS = Double.NaN
        entryAngleDegrees = Double.NaN
        vertexHeightCm = Double.NaN
        vertexDistanceInches = Double.NaN
        flightTimeSeconds = Double.NaN
        ballExitVelocityMps = Double.NaN
        entryAngleErrorDegrees = Double.NaN
        heightConstraintSatisfied = false
        feasible = false
        valid = false
        status = "Outside 35–75 in or invalid distance; no shot"
    }
}

/** Experimental system only. Production Flywheel/TeleOp never reference this object. */
object PhysicsShooter {
    const val MIN_DISTANCE_IN = 35.0
    const val MAX_DISTANCE_IN = 75.0
    const val TARGET_HEIGHT_CM = PhysicsShotTable.TARGET_HEIGHT_CM
    const val PREFERRED_ENTRY_DEGREES = PhysicsShotTable.PREFERRED_ENTRY_DEG
    const val REQUIRED_VERTEX_CM = 136.0 // 135 cm + 1 cm margin
    const val CALIBRATION_PROVISIONAL = true
    const val PWM_MIN_US = PhysicsShotTable.PWM_MIN_US
    const val PWM_MAX_US = PhysicsShotTable.PWM_MAX_US

    /** O(1) uniform-grid lookup, no allocation, integration, search, or hardware access. */
    @JvmStatic
    fun calculateShot(horizontalDistanceInches: Double, result: PhysicsShot): PhysicsShot {
        if (!horizontalDistanceInches.isFinite() || horizontalDistanceInches < MIN_DISTANCE_IN ||
            horizontalDistanceInches > MAX_DISTANCE_IN) {
            result.invalidate()
            return result
        }
        val index = (horizontalDistanceInches - MIN_DISTANCE_IN) / PhysicsShotTable.STEP_IN
        val low = floor(index).toInt().coerceIn(0, PhysicsShotTable.distance.lastIndex)
        val high = minOf(low + 1, PhysicsShotTable.distance.lastIndex)
        val fraction = index - low
        result.hoodAngleDegrees = lerp(PhysicsShotTable.hood, low, high, fraction)
        result.launchAngleDegrees = 90.0 - result.hoodAngleDegrees + PhysicsShotTable.ANGLE_OFFSET_DEG
        result.shooterVelocityTPS = lerp(PhysicsShotTable.tps, low, high, fraction)
        result.entryAngleDegrees = lerp(PhysicsShotTable.entry, low, high, fraction)
        result.vertexHeightCm = lerp(PhysicsShotTable.vertexCm, low, high, fraction)
        result.vertexDistanceInches = lerp(PhysicsShotTable.vertexIn, low, high, fraction)
        result.flightTimeSeconds = lerp(PhysicsShotTable.flight, low, high, fraction)
        result.ballExitVelocityMps = lerp(PhysicsShotTable.speed, low, high, fraction)
        result.entryAngleErrorDegrees = abs(result.entryAngleDegrees - PREFERRED_ENTRY_DEGREES)
        // Conservative at a feasibility boundary; do not interpolate booleans into a claim.
        result.heightConstraintSatisfied = PhysicsShotTable.feasible[low] &&
            (fraction == 0.0 || PhysicsShotTable.feasible[high]) &&
            result.vertexHeightCm >= REQUIRED_VERTEX_CM
        result.feasible = result.heightConstraintSatisfied
        result.valid = true // descending fallback is a valid trajectory, but not height-feasible
        result.status = when {
            result.feasible -> "Provisional model: constraints satisfied"
            !PhysicsShotTable.feasible[low] && !PhysicsShotTable.feasible[high] ->
                "Height infeasible: highest-vertex descending fallback"
            else -> "Conservative feasibility boundary: fallback estimate"
        }
        return result
    }

    /** User-confirmed linear servo geometry: 355 deg * 90% PWM span * 30/148 gearing. */
    @JvmStatic
    fun hoodServoPosition(hoodAngleDegrees: Double): Double {
        if (!hoodAngleDegrees.isFinite() || hoodAngleDegrees < 15.0 || hoodAngleDegrees > 36.0)
            return Double.NaN
        return (0.485 - (hoodAngleDegrees-15.0)/PhysicsShotTable.HOOD_DEGREES_PER_POSITION)
            .coerceIn(0.16, 0.485)
    }

    @JvmStatic
    fun hoodDegreesFromServoPosition(position: Double): Double {
        if (!position.isFinite() || position < hoodServoPosition(36.0) || position > 0.485)
            return Double.NaN
        return 15.0 + (0.485-position)*PhysicsShotTable.HOOD_DEGREES_PER_POSITION
    }

    private fun lerp(values: DoubleArray, low: Int, high: Int, fraction: Double): Double =
        values[low] + fraction*(values[high]-values[low])
}

/** Analytic baseline for debugging. All distances in meters, angles in degrees. */
object IdealProjectile {
    const val GRAVITY = 9.80665

    @JvmStatic
    fun requiredSpeed(distanceM: Double, heightM: Double, launchAngleDegrees: Double): Double {
        if (!distanceM.isFinite() || distanceM <= 0 || !heightM.isFinite() ||
            !launchAngleDegrees.isFinite() || launchAngleDegrees <= 0 || launchAngleDegrees >= 90)
            return Double.NaN
        val theta = Math.toRadians(launchAngleDegrees)
        val denominator = 2*cos(theta).pow(2)*(distanceM*tan(theta)-heightM)
        return if (denominator > 0) sqrt(GRAVITY*distanceM*distanceM/denominator) else Double.NaN
    }
}
