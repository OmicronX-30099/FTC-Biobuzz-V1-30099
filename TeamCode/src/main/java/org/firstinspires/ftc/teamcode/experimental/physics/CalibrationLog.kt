package org.firstinspires.ftc.teamcode.experimental.physics

import java.util.Locale
import kotlin.math.hypot

/** Buffer feed attempts in memory; serialize after motors have stopped, never each control loop. */
class CalibrationLog {
    private data class Attempt(val values: DoubleArray, val goal: Int, val settled: Boolean,
                               var result: String = "UNRATED")
    private val attempts = ArrayList<Attempt>(128)
    val count: Int get() = attempts.size
    var scores = 0
        private set
    var misses = 0
        private set
    val unrated: Int get() = count-scores-misses

    /** Called before opening the gate, with the current encoder velocity, not post-shot sag. */
    fun capture(timeSeconds: Double, robotX: Double, robotY: Double, headingRad: Double,
                goal: Int, goalX: Double, goalY: Double, targetTPS: Double,
                measuredTPS: Double, hoodPosition: Double, settled: Boolean): Boolean {
        val hoodDegrees = PhysicsShooter.hoodDegreesFromServoPosition(hoodPosition)
        val distance = hypot(goalX-robotX, goalY-robotY)
        val values = doubleArrayOf(timeSeconds, robotX, robotY, headingRad, goalX, goalY,
            distance, targetTPS, measuredTPS, hoodPosition, hoodDegrees)
        if (values.any { !it.isFinite() } || distance <= 0 || targetTPS <= 0 || goal !in 1..2)
            return false
        attempts.add(Attempt(values, goal, settled))
        return true
    }

    /** Rate only the latest pending attempt; a held button cannot label several attempts. */
    fun rateLast(scored: Boolean): Boolean {
        val last = attempts.lastOrNull() ?: return false
        if (last.result != "UNRATED") return false
        last.result = if (scored) "SCORE" else "MISS"
        if (scored) scores++ else misses++
        return true
    }

    fun toCsv(): String {
        val text = StringBuilder("attempt,time_s,robot_x_in,robot_y_in,heading_rad,goal,goal_x_in,goal_y_in,distance_in,target_motor_tps,measured_pre_feed_motor_tps,hood_servo,hood_degrees,settled,result,target_height_cm,pwm_min_us,pwm_max_us\n")
        attempts.forEachIndexed { i, attempt ->
            val v = attempt.values
            text.append(i+1).append(',')
            for (j in 0..3) text.append(String.format(Locale.US, "%.6f,", v[j]))
            text.append(attempt.goal).append(',')
            for (j in 4..10) text.append(String.format(Locale.US, "%.6f,", v[j]))
            text.append(attempt.settled).append(',').append(attempt.result).append(',')
                .append(String.format(Locale.US, "%.1f,%.1f,%.1f\n",
                    PhysicsShooter.TARGET_HEIGHT_CM, PhysicsShooter.PWM_MIN_US, PhysicsShooter.PWM_MAX_US))
        }
        return text.toString()
    }
}
