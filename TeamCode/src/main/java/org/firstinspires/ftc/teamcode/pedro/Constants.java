package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("flDrive");
                c.backLeftName.set("blDrive");
                c.frontRightName.set("frDrive");
                c.backRightName.set("brDrive");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(-6.64);
                c.yPodOffset.set(-1.87);
                // Pod axes are robot-relative: forward and lateral, respectively.
                // At the 90-degree starting heading, forward must increase field Y
                // and moving right must increase field X.
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.31685793720585537);
                Controller secondaryTranslationalForward = Controller.proportional(0.11707054119754981);
                Controller primaryTranslationalLateral = Controller.proportional(0.4265819035271024);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15761061487487335);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01260002603625443));
                c.brake.set(Controller.proportionalFeedforward(0.010710022130816265));

                c.headingFeedback.set(Controller.proportional(5.3052046161608635));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05188400734787066, 0.004656723864122758));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10284716496274006, 0.0651461734427604));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010568967801026, 0.0014896109603780466));

                c.maxAchievableForwardVelocity.set(75.46215246397416);
                c.maxAchievableStrafeVelocity.set(60.39054311773454);
                c.naturalForwardDeceleration.set(23.78538663513537);
                c.naturalStrafeDeceleration.set(63.59765654300642);
            }
    );

    public static Follower create(HardwareMap h) {
        PinpointLocalizer localizer = new PinpointLocalizer(h, localizerConfig);
        Mecanum drivetrain = new Mecanum(h, drivetrainConfig);
        // Pedro initializes in FLOAT; enable BRAKE during INIT as well as manual driving.
        drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return new Follower(
                localizer,
                drivetrain,
                new Foresight(foresightConfig));
    }
}
