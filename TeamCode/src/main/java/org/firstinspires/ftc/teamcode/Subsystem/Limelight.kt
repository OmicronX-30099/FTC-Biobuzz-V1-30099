package org.firstinspires.ftc.teamcode.Subsystem

import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.sun.tools.doclint.Entity
import org.firstinspires.ftc.teamcode.Subsystem.Limelight.limeLight
import java.util.Locale
import java.util.Locale.getDefault
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

object Limelight {
    const val CIRCLE_PIPELINE = 3
    const val APRIL_PIPELINE = 1
    var CURRENT_PIPELINE = 0;
    // 1st: Target X Degrees, 2nd: Target Y Degrees, Third: Target Area
    lateinit var CIRCLE_RESULTS: Triple<Double, Double, Double>
    lateinit var limeLight: Limelight3A



    fun init(hardwareMap: HardwareMap) {
        limeLight = hardwareMap.get(Limelight3A::class.java, "limeL")
    }
    fun SwitchPipeline(input: String) {
        val lowercase = input.lowercase(getDefault())
        if (lowercase == "ball" || lowercase == "b") {
            CURRENT_PIPELINE = CIRCLE_PIPELINE
            limeLight.pipelineSwitch(CIRCLE_PIPELINE)
        }
        else if (lowercase == "april" || lowercase == "a") {
            CURRENT_PIPELINE = APRIL_PIPELINE
            limeLight.pipelineSwitch(APRIL_PIPELINE)
        }
    }
    fun DetectCircle() {
        val result = limeLight.latestResult
        if (!result.isValid || result == null) return

        val detection = result.detectorResults
        val listOfNectar: MutableList<LLResultTypes.DetectorResult> = mutableListOf()

        for (i in detection) {
            if (i.className == "yellow_pollen") {
                listOfNectar.add(i)
            }
        }

        val largeTarget = listOfNectar.maxByOrNull{it.targetArea}
        if (largeTarget != null)
            CIRCLE_RESULTS = Triple(largeTarget.targetXDegrees , largeTarget.targetYDegrees, largeTarget.targetArea)
        return
    }
    fun CalcPose(follower: Follower): Pose {
        val CAMERA_ANGLE_OFFSET: Double = 0.0
        val HEIGHT_DIFFERENCE: Double = 0.0
        // USE ONLY FOR COLOR BLOBS
        val height = HEIGHT_DIFFERENCE
        val angle = CAMERA_ANGLE_OFFSET - CIRCLE_RESULTS.second
        val horiDistance = height/tan(angle * (Math.PI/180))

        val ballAngle = follower.pose().heading() +  CIRCLE_RESULTS.first * (Math.PI/180)
        val ballCoordX = follower.pose().x() + horiDistance * cos(ballAngle)
        val ballCoordY = follower.pose().y() + horiDistance * sin(ballAngle)

        return Pose(ballCoordX, ballCoordY, ballAngle)


    }

}