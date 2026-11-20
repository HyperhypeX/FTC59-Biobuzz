package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.util.RobotConstants;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConstants.Drivetrain.leftFront);
        c.backLeftName.set(RobotConstants.Drivetrain.leftRear);
        c.frontRightName.set(RobotConstants.Drivetrain.rightFront);
        c.backRightName.set(RobotConstants.Drivetrain.rightRear);
        // Same values RobotHardware applies for TeleOp - edit them in RobotConstants.
        c.frontLeftDirection.set(RobotConstants.Drivetrain.leftFrontDirection);
        c.backLeftDirection.set(RobotConstants.Drivetrain.leftRearDirection);
        c.frontRightDirection.set(RobotConstants.Drivetrain.rightFrontDirection);
        c.backRightDirection.set(RobotConstants.Drivetrain.rightRearDirection);
    });

    // Measured with the Pinpoint tuner (offsets in inches).
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(7.797023743156374);
        c.yPodOffset.set(0.7795723592202495);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED); // flipped after hand-push test: forward push made y fall
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // Measured with the Foresight tuner.
    public static ForesightConfig algorithmConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.3023734629664848);
        Controller secondaryTranslationalForward = Controller.proportional(0.11171891499838214);
        Controller primaryTranslationalLateral = Controller.proportional(0.4116317237381095);
        Controller secondaryTranslationalLateral = Controller.proportional(0.15208692291900164);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.01282595354195507));
        c.brake.set(Controller.proportionalFeedforward(0.01090206051066181));

        c.headingFeedback.set(Controller.proportional(5.059133976165648));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05205129161372111, 0.008616631849083145));

        c.linearBrakeCoefficients.set(Matrix.diag(0.07380165328326548, 0.03370186497338689));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021112509620110557, 0.002587365243297366));

        c.maxAchievableForwardVelocity.set(73.6615272685018);
        c.maxAchievableStrafeVelocity.set(63.25025984858282);
        c.naturalForwardDeceleration.set(31.310365289944336);
        c.naturalStrafeDeceleration.set(66.69994028372751);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(algorithmConfig));
    }
}
