package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.follower.Follower;
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
        // TODO: Check whether these reverses are correct
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // Offsets carried over from the 2.x config (forwardPodY -> xPodOffset, strafePodX -> yPodOffset).
    // Re-run the Pinpoint Tuner to confirm: it prints this whole block with measured values.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.5);
        c.yPodOffset.set(2.5);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // P/I/D carried over from the 2.x coefficients, whose F terms were all 0 so nothing is lost.
    // TODO: mass, xVelocity/yVelocity and the zero-power accelerations have no 3.0.0 equivalent.
    // Run the tuners for maxAchievableForwardVelocity / naturalForwardDeceleration and friends.
    public static ForesightConfig algorithmConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(new PIDController(0.3, 0, 0.05));
        c.strafeTranslational.set(new PIDController(0.3, 0, 0.05));
        c.headingFeedback.set(new PIDController(0.7, 0, 0.01));
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(algorithmConfig));
    }
}
